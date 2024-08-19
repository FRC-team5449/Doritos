// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.commands;

import com.team5449.frc2024.RobotContainer;
import com.team5449.frc2024.subsystems.score.Intake;
import com.team5449.frc2024.subsystems.score.Shooter;
import com.team5449.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
  private final Intake mIntake;
  private final Shooter mShooter;
  private final ArmPoseCommand mArm;
  private final boolean isRaw;
  private final TimeDelayedBoolean mFinishedFlag = new TimeDelayedBoolean();
  /** Creates a new IntakeCommand. */
  public IntakeCommand(Shooter shooter, Intake intake, ArmPoseCommand arm, boolean isRaw) {
    mIntake = intake;
    mShooter = shooter;
    mArm = arm;
    this.isRaw = isRaw;
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!isRaw){
      mArm.setPose(ArmPoseCommand.ArmSystemState.INTAKE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isRaw || mArm.getArmState() == ArmPoseCommand.ArmSystemState.INTAKE){
      mIntake.setIntakeSpeed(1);
      mShooter.setOpenLoop(-0.2, true);
      mShooter.transit(-0.8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.setIntakeSpeed(0);
    mShooter.setOpenLoop(0, false);
    mShooter.transit(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mFinishedFlag.update(RobotContainer.noteStored.get(), 0.15) && !isRaw;
  }
}
