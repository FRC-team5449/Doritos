package com.team5449.frc2024.commands;

import java.util.function.BooleanSupplier;

import com.team5449.frc2024.subsystems.score.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AmpCommand extends Command{
private final Shooter mShooter;
  private final BooleanSupplier isArmSet;
  private double speed;
  public AmpCommand(Shooter shooter, BooleanSupplier isArmPositionSet,double s) {
    mShooter = shooter;
    isArmSet = isArmPositionSet;
    speed=s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.setAmpShooting(speed);
    
    SmartDashboard.putBoolean("isShooterAMP", mShooter.isShooterAtSetpoint());
    if(mShooter.isShooterAtSetpoint() && isArmSet.getAsBoolean())
    {
      mShooter.transit(1);
    }
    else{
      mShooter.transit(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setOpenLoop(0, false);
    mShooter.transit(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
