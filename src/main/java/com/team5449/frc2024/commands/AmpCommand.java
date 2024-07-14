package com.team5449.frc2024.commands;

import java.util.function.BooleanSupplier;

import com.team5449.frc2024.subsystems.score.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AmpCommand extends Command{
  private final Shooter mShooter;
  private final BooleanSupplier isArmSet;
  private final ArmPoseCommand mArm;

  private boolean isTransitRunning;
  private boolean isNoteOuted;
  private final boolean isRaw;

  private double speed;
  public AmpCommand(Shooter shooter, ArmPoseCommand mArm, BooleanSupplier isArmPositionSet,double s, boolean isRaw) {
    mShooter = shooter;
    isArmSet = isArmPositionSet;
    this.mArm = mArm;
    this.isRaw = isRaw;
    speed=s;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!isRaw){
      mArm.setPose(ArmPoseCommand.ArmSystemState.AMP);
      isTransitRunning = false;
      isNoteOuted = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.setAmpShooting(speed);
    
    SmartDashboard.putBoolean("isShooterAMP", mShooter.isShooterAtSetpoint());
    if(!mShooter.isShooterAtSetpoint() && isTransitRunning){
      mShooter.transit(0);
      isTransitRunning = false;
      isNoteOuted = true;
      System.out.println("AMP!!!!");
    }
    if(mShooter.isShooterAtSetpoint() && isArmSet.getAsBoolean())
    {
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
    mShooter.setAmpShooting(0);
    mShooter.transit(0);
    if(isNoteOuted == true && !isRaw){
      mArm.setPose(ArmPoseCommand.ArmSystemState.INTAKE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
