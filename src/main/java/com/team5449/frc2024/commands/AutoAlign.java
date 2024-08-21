// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.commands;

import com.team5449.frc2024.FieldLayout;
import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;
import com.team5449.frc2024.subsystems.vision.VisionSubsystem;
import com.team5449.lib.util.GeomUtil;
import com.team5449.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign extends Command {
  private final DrivetrainSubsystem mDrive;
  // private final VisionSubsystem mVision;
  private final PIDController omegaController = new PIDController(3, 0, 0);
  private double lastv = 0;
  private final double maxAccelerationRadPerSecSq = 2;
  private final double maxVelocityRadPerSec = 4;
  private double lastt = 0;
  private final Field2d mOutput = new Field2d();

  /** Creates a new AutoAlign. */
  public AutoAlign(DrivetrainSubsystem drive, VisionSubsystem vision) {
    mDrive = drive;
    // mVision = vision;
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    SmartDashboard.putData("Drive/StagePose", mOutput);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // calcTargetAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    calcTargetAngle();

    // if(mVision.getTargetDetected(0)){
    //   double omegaVelocity = -mVision.getOffsetToTarget(0);
    //   SmartDashboard.putNumber("Omega Velocity", Units.degreesToRadians(omegaVelocity));
    //   mDrive.setHeadingControlSpeed(omegaVelocity);
    // } else {
      double thist = Timer.getFPGATimestamp();
      double dt = thist - lastt;
      double omegaVelocity = omegaController.calculate(mDrive.getHeading().getRadians());
      double RadAccleration = (omegaVelocity-lastv)/dt;
      
      SmartDashboard.putNumber("Auto Align/PID Velocity", omegaVelocity);
      if(Math.abs(RadAccleration)>Math.abs(maxAccelerationRadPerSecSq)){
        omegaVelocity = lastv+Math.copySign(maxAccelerationRadPerSecSq, RadAccleration)*dt;
      }
      if(Math.abs(omegaVelocity)>Math.abs(maxVelocityRadPerSec)){
        omegaVelocity = Math.copySign(maxVelocityRadPerSec, omegaVelocity);
      }
      lastv = omegaVelocity;
      lastt = thist;
      // double slewRateVelocity = mLimiter.calculate(omegaVelocity);
      mDrive.setHeadingControlSpeed(omegaVelocity);

      SmartDashboard.putNumber("Auto Align/Slew Rate Velocity", omegaVelocity);
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // mDrive.setTargetVelocity(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return /*mVision.getTargetDetected(0) ? Util.epsilonEquals(0, mVision.getOffsetToTarget(0), 3.0) :*/ omegaController.atSetpoint();
  //return false;
  }
  

  private void calcTargetAngle() {
    // double adjustAngleToTartget = mDrive.getHeading().getDegrees();
    // Pose2d currentPose = mDrive.getEstimatedPose();
    // double botToTargetX = 0;
    // double botToTargetY = 0;

    // if(DriverStation.getAlliance().get() == Alliance.Blue){
    //   botToTargetX = FieldLayout.BlueAllianceSpeaker.getX() - currentPose.getX();
    //   botToTargetY = FieldLayout.BlueAllianceSpeaker.getY() - currentPose.getY();
    // }
    // else{
    //   botToTargetX = FieldLayout.RedAllianceSpeaker.getX() - currentPose.getX();
    //   botToTargetY = FieldLayout.RedAllianceSpeaker.getY() - currentPose.getY();
    // }

    // adjustAngleToTartget = Math.atan2(botToTargetY, botToTargetX);
    // if(DriverStation.getAlliance().get() == Alliance.Red)
    // {
    //   adjustAngleToTartget += Math.PI;
    // }
    // omegaController.setSetpoint(adjustAngleToTartget);

    // SmartDashboard.putNumber("Adjust Angle", Units.radiansToDegrees(adjustAngleToTartget));
    mOutput.setRobotPose(new Pose2d(GeomUtil.GetStageTranslation().toTranslation2d(), new Rotation2d()));
    double rotation = GeomUtil.GetStageTranslation().toTranslation2d().minus(mDrive.getPose().getTranslation()).getAngle().getRadians();
    SmartDashboard.putNumber("Auto Align/Adjust Angle", Units.radiansToDegrees(rotation));
    omegaController.setSetpoint(rotation);
  }
}
