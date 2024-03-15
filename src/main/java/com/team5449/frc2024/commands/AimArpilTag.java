// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.commands;

import com.team5449.frc2024.Constants;
import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;
import com.team5449.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimArpilTag extends ProfiledPIDCommand {
  private final DrivetrainSubsystem mDrive;
  /** Creates a new AimArpilTag. */
  public AimArpilTag(DrivetrainSubsystem drive, VisionSubsystem vision) {
    super(
        new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(Constants.maxAutoVelocityMeterPerSec, Constants.maxAutoAccelerationMeterPerSecSq)),
        () -> 0,
        () -> new TrapezoidProfile.State(),

        (output, setpoint) -> {
          
        });

    mDrive = drive;
    addRequirements(mDrive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void end(boolean interrupted) {
    mDrive.setTargetVelocity(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
