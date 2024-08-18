// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.team5449.frc2024.FieldLayout;
import com.team5449.frc2024.subsystems.CommandSwerveDrivetrain;
import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;
import com.team5449.frc2024.subsystems.vision.VisionSubsystem;
import com.team5449.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign extends Command {
  private final CommandSwerveDrivetrain mDrive;
  private final VisionSubsystem mVision;
  private final PIDController omegaController = new PIDController(3, 0, 0);

  /** Creates a new AutoAlign. */
  public AutoAlign(CommandSwerveDrivetrain drive, VisionSubsystem vision) {
    mDrive = drive;
    mVision = vision;
    omegaController.enableContinuousInput(0, 2 * Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calcTargetAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //calcTargetAngle();

    // if(mVision.getTargetDetected(0)){
    //   double omegaVelocity = -mVision.getOffsetToTarget(0);
    //   SmartDashboard.putNumber("Omega Velocity", Units.degreesToRadians(omegaVelocity));
    //   mDrive.setHeadingControlSpeed(omegaVelocity);
    // } else {
      double omegaVelocity = omegaController.calculate(mDrive.getState().Pose.getRotation().getRadians());
      // mDrive.setHeadingControlSpeed(omegaVelocity);
      mDrive.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(0,0, omegaVelocity)));
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // mDrive.setTargetVelocity(new ChassisSpeeds(0, 0, 0));
    mDrive.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mVision.getTargetDetected(0) ? Util.epsilonEquals(0, mVision.getOffsetToTarget(0), 3.0) : omegaController.atSetpoint();
  //return false;
  }
  

  private void calcTargetAngle() {
    double adjustAngleToTartget = mDrive.getState().Pose.getRotation().getDegrees();
    Pose2d currentPose = mDrive.getState().Pose;
    double botToTargetX = 0;
    double botToTargetY = 0;

    if(DriverStation.getAlliance().get() == Alliance.Blue){
      botToTargetX = FieldLayout.BlueAllianceSpeaker.getX() - currentPose.getX();
      botToTargetY = FieldLayout.BlueAllianceSpeaker.getY() - currentPose.getY();
    }
    else{
      botToTargetX = FieldLayout.RedAllianceSpeaker.getX() - currentPose.getX();
      botToTargetY = FieldLayout.RedAllianceSpeaker.getY() - currentPose.getY();
    }

    adjustAngleToTartget = Math.atan2(botToTargetY, botToTargetX);
    if(DriverStation.getAlliance().get() == Alliance.Red)
    {
      adjustAngleToTartget += Math.PI;
    }
    omegaController.setSetpoint(adjustAngleToTartget);

    SmartDashboard.putNumber("Adjust Angle", Units.radiansToDegrees(adjustAngleToTartget));
  }
}
