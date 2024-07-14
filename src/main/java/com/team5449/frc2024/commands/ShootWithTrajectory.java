package com.team5449.frc2024.commands;

import com.team5449.frc2024.commands.ArmPoseCommand.ArmSystemState;
import com.team5449.frc2024.subsystems.score.Arm;
import com.team5449.frc2024.subsystems.score.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootWithTrajectory extends Command {
  private final Shooter mShooter;
  private final ArmPoseCommand mArm;
  private final Translation2d target;
  private double velocity;
  private static final double vRatio = 10/Math.PI*0.75;
  /** Creates a new ShootWithTrajectory. */
  public ShootWithTrajectory(Shooter shooter, ArmPoseCommand mArmPoseCommand, Translation2d targetPt) {
    mShooter = shooter;
    mArm = mArmPoseCommand;
    target = targetPt;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mArm.setPose(ArmSystemState.SHOOTING);
    ArmSystemState.SHOOTING.armPose = Arm.ManualOffset + 1/8;
    velocity = Math.sqrt(9.8 * target.getX()); // m/s
    mShooter.setShootRPM(velocity*vRatio/0.6); // 胶轮直径： 100mm，齿轮比24->18(3:4)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mArm.getArmState() == ArmSystemState.SHOOTING && mShooter.isShooterAtSetpoint()){
      mShooter.transit(1);
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
