// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.commands;

import java.util.function.BooleanSupplier;

import com.team5449.frc2024.commands.ArmPoseCommand.ArmSystemState;
import com.team5449.frc2024.subsystems.score.Shooter;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCommand extends Command {
  private final Shooter mShooter;
  private final ArmPoseCommand mArm;
  private final BooleanSupplier isArmSet;
  private final BooleanConsumer onShoot;

  private boolean isTransitRunning;
  private boolean isNoteOuted;

  private double shooterSetpoint;
  public ShootCommand(Shooter shooter, ArmPoseCommand mArmPoseCommand, BooleanSupplier isArmPositionSet, double setpoint, BooleanConsumer onShoot) {
    mShooter = shooter;
    isArmSet = isArmPositionSet;
    shooterSetpoint = setpoint;
    this.onShoot = onShoot;
    mArm = mArmPoseCommand;
    addRequirements(mShooter);
  }
  public ShootCommand(Shooter shooter, ArmPoseCommand mArmPoseCommand, BooleanSupplier isArmPositionSet, double setpoint) {
    this(shooter, mArmPoseCommand, isArmPositionSet, setpoint, (e)->{});
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isTransitRunning = false;
    isNoteOuted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    mShooter.setShootRPM(shooterSetpoint);

    SmartDashboard.putBoolean("Shooter/isAtSetpoint", mShooter.isShooterAtSetpoint());
    SmartDashboard.putBoolean("Shooter/isTransitRunning", isTransitRunning);
    SmartDashboard.putBoolean("Shooter/isNoteOuted", isNoteOuted);

    if(!mShooter.isShooterAtSetpoint() && isTransitRunning)
    {
      mShooter.transit(0);
      isTransitRunning = false;
      isNoteOuted = true;
      System.out.println("Shoot!!!");
      onShoot.accept(isNoteOuted);
    }
    if(mShooter.isShooterAtSetpoint() && isArmSet.getAsBoolean() /*&& !isTransitRunning*/){
      mShooter.transit(1);
      isTransitRunning = true;
    }
    else{
      mShooter.transit(0);
      isTransitRunning = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setOpenLoop(0, false);
    mShooter.transit(0);
    if(isNoteOuted){
      mArm.setPose(ArmSystemState.INTAKE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isNoteOuted; // TODO: test if works
  }
  public boolean isNoteShooted()
  {
    return isNoteOuted;
  }
}
