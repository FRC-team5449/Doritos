// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.commands;

import java.util.function.BooleanSupplier;

import com.team5449.frc2024.subsystems.score.Shooter;
import com.team5449.frc2024.subsystems.score.ShooterFalcon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShaoShooterCommand extends Command {
  private final ShooterFalcon mShooter;
  private final BooleanSupplier isArmSet;
  private double shooterSetpoint;
  public ShaoShooterCommand(ShooterFalcon shooter, BooleanSupplier isArmPositionSet, double setpoint) {
    mShooter = shooter;
    isArmSet = isArmPositionSet;
    shooterSetpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    mShooter.setShootRPM(shooterSetpoint);

    SmartDashboard.putBoolean("isShooterAtSetpoint", mShooter.isShooterAtSetpoint(shooterSetpoint));

    if(mShooter.isShooterAtSetpoint(shooterSetpoint) && isArmSet.getAsBoolean()){
      mShooter.transit(0.9);
    }
    else{
      mShooter.transit(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setShootRPM(0);
    mShooter.transit(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
