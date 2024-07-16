package com.team5449.frc2024.commands;

import java.util.function.Supplier;

import com.team5449.frc2024.commands.ArmPoseCommand.ArmSystemState;
import com.team5449.frc2024.subsystems.score.Arm;
import com.team5449.frc2024.subsystems.score.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootWithTrajectory extends Command {
  private final Shooter mShooter;
  private final ArmPoseCommand mArm;
  private final Supplier<Translation2d> target;
  private double velocity;
  private Rotation2d theta;
  private static final double vRatio = 10/Math.PI*0.75;
  /** Creates a new ShootWithTrajectory. */
  public ShootWithTrajectory(Shooter shooter, ArmPoseCommand mArmPoseCommand, Supplier<Translation2d> targetPt) {
    mShooter = shooter;
    mArm = mArmPoseCommand;
    target = targetPt;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onVarUpdate();
  }

  private void calcEssentials(){
    Translation2d mtarget = target.get();
    SmartDashboard.putNumber("TargetX", mtarget.getX());
    double vx,b;
    theta = mtarget.getAngle().rotateBy(Rotation2d.fromDegrees(90)).div(2);
    // mtarget = mtarget.minus(new Translation2d(0.565, theta).plus(new Translation2d(-0.25, 0.65)));
    // theta = mtarget.getAngle().rotateBy(Rotation2d.fromDegrees(90)).div(2);
    System.out.print("theta=");
    System.out.print(theta);
    System.out.print(";b=");
    b = theta.getTan();
    System.out.print(b);
    System.out.print(";targetAngle=");
    System.out.print(mtarget.getAngle());
    vx = Math.sqrt(mtarget.getX()*(-9.8*0.5) / (mtarget.getAngle().getTan() - b));
    velocity = vx / theta.getCos();
    System.out.print(";vx=");
    System.out.print(vx);
    System.out.print(";v=");
    System.out.println(velocity);
    velocity = velocity*vRatio*(1+mtarget.getX()/10);
    //velocity = Math.sqrt(9.8 * mtarget.getX()); // m/s
  }

  private void setOutput(){
    mArm.setPose(ArmSystemState.SHOOTING);
    ArmSystemState.SHOOTING.armPose = Arm.ManualOffset + theta.getRotations();
    mShooter.setShootRPM(velocity); // 胶轮直径： 100mm，齿轮比24->18(3:4)
  }

  private void onVarUpdate(){
    calcEssentials();
    setOutput();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    onVarUpdate();
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
