// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.autos.autocommands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveTrajectoryCommand extends Command {
  private static final double ACCELERATION_COEFFICIENT = 0.2;

  public static final double ALLOWABLE_POSE_ERROR = 0.05;
  public static final double ALLOWABLE_ROTATION_ERROR = Math.toRadians(2);

  /**
   * Timer object
   */
  private final Timer timer = new Timer();

  /**
   * Drivetrain object to access subsystem.
   */
  private DrivetrainSubsystem drivetrainSubsystem;
  /**
   * Path Planner trajectory to follow
   */
  private PathPlannerTrajectory trajectory;

  private State previousState;

  /**
   * The auto balance on charge station command constructor.
   *
   * @param drivetrainSubsystem The coordinator between the gyro and the swerve modules.
   * @param trajectory          The trajectory to follow.
   */
  public DriveTrajectoryCommand(DrivetrainSubsystem drivetrainSubsystem, PathPlannerTrajectory trajectory) {
      this.drivetrainSubsystem = drivetrainSubsystem;
      this.trajectory = trajectory;
      addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
      // Reset and begin timer
      this.timer.reset();
      this.timer.start();
      // Logger.getInstance().recordOutput("Drivetrain/Trajectory", trajectory);
      // Get initial state of path
      previousState = trajectory.getInitialState();
  }

  @Override
  public void execute() {
      double currentTime = this.timer.get();
      State desiredState =
              trajectory.sample(currentTime);


      //desiredState = PathPlannerTrajectory.transformStateForAlliance(desiredState, DriverStation.getAlliance());
      Rotation2d heading = desiredState.heading;


      ChassisSpeeds chassisSpeeds =
              drivetrainSubsystem.getSwerveFollower().calculateRobotRelativeSpeeds(drivetrainSubsystem.getPose(), desiredState);
      chassisSpeeds.vxMetersPerSecond +=
              desiredState.accelerationMpsSq * heading.getCos() * ACCELERATION_COEFFICIENT;
      chassisSpeeds.vyMetersPerSecond +=
              desiredState.accelerationMpsSq * heading.getSin() * ACCELERATION_COEFFICIENT;

      drivetrainSubsystem.setTargetVelocity(chassisSpeeds);
      Logger.recordOutput(
                      "Desired Auto Pose",
                      new Pose2d(desiredState.positionMeters, desiredState.heading));
  }

  @Override
  public void end(boolean interrupted) {
      this.timer.stop(); // Stop timer
      drivetrainSubsystem.setTargetVelocity(new ChassisSpeeds()); // Stop motors
  }

  @Override
  public boolean isFinished() {
      // Finish command if the total time the path takes is over
      double driveX = drivetrainSubsystem.getPose().getX();
      double driveY = drivetrainSubsystem.getPose().getY();
      double driveRotation = drivetrainSubsystem.getPose().getRotation().getRadians();

      double desiredX = trajectory.getEndState().positionMeters.getX();
      double desiredY = trajectory.getEndState().positionMeters.getY();
      double desiredRotation =
              trajectory.getEndState().heading.getRadians();

      double xError = Math.abs(desiredX - driveX);
      double yError = Math.abs(desiredY - driveY);
      double rotationError = Math.abs(desiredRotation - driveRotation);

      return (xError < ALLOWABLE_POSE_ERROR
                      && yError < ALLOWABLE_POSE_ERROR
                      && rotationError < ALLOWABLE_ROTATION_ERROR)
              || timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
