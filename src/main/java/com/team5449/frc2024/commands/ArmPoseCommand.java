package com.team5449.frc2024.commands;


import java.nio.ShortBuffer;

import com.ctre.phoenix6.signals.System_StateValue;
import com.team5449.frc2024.subsystems.score.Arm;
import com.team5449.frc2024.subsystems.vision.VisionSubsystem;
import com.team5449.lib.CConsole;
import com.team5449.lib.interpolate.InterpolatingDouble;
import com.team5449.lib.interpolate.InterpolatingTreeMap;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmPoseCommand extends Command{
private final Arm mArm;
private final VisionSubsystem mVision;
private final ShuffleboardTab mTab = Shuffleboard.getTab("Arm");
private final GenericEntry mDist = mTab.add("Robot relative dist", -1).getEntry();

private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShooterRPMTreeMap = new InterpolatingTreeMap<>();

private double offset = 0;

static{
  mShooterRPMTreeMap.put(new InterpolatingDouble(1.533),new InterpolatingDouble(0.235));
  mShooterRPMTreeMap.put(new InterpolatingDouble(1.83),new InterpolatingDouble(0.23));
  mShooterRPMTreeMap.put(new InterpolatingDouble(1.99),new InterpolatingDouble(0.22));
  mShooterRPMTreeMap.put(new InterpolatingDouble(2.33),new InterpolatingDouble(0.21));
  mShooterRPMTreeMap.put(new InterpolatingDouble(2.70),new InterpolatingDouble(0.2));
  
}

  public ArmPoseCommand(Arm arm, VisionSubsystem vision) {
    mArm = arm;
    mVision = vision;
    addRequirements(mArm, mVision);
  }

  public void setPose(Arm.ArmSystemState newSystemState){
    mArm.setPose(newSystemState);
  }

  public void setAutoShootPosition(double position){
    mArm.setAutoShootPosition(position);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double botToTarget = mVision.getStageDistance(0);
    SmartDashboard.putNumber("Dist", botToTarget);

    if(mArm.getArmState() == Arm.ArmSystemState.SHOOTING){
        
      //botToTarget = -2;
    if(botToTarget < 0){
      mArm.setShootArmPose(0.148+offset);// mSystemState.armPose = 0.148;
    }
    else{
      //double armPose = Units.radiansToRotations(Math.atan((com.team5449.frc2024.Constants.armToSpeakerVerticalMeter) / Math.abs(botToTarget)));
      //mSystemState.armPose = armPose + 0.11;
      //mSystemState.armPose = shootingArmPose(botToTarget);
      mArm.setShootArmPose(shootingArmPose(botToTarget)+offset);
      //CConsole.stdout.log("Setted armpose", mSystemState, "= shootingArmPose(", botToTarget, ") =", mSystemState.armPose);
    }
    //mSystemState.armPose += offset;

  // if(mPos.getDouble(mSystemState.armPose)!=mSystemState.armPose)
  // {
  //   mSystemState.armPose = mPos.getDouble(0);
  // }
  }
    


    //SmartDashboard.putNumber("Arm Pose", mSystemState.armPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
  public Arm.ArmSystemState getArmState()
  {
    return mArm.getArmState();
  }

  public void offsetBy(double offset)
  {
    if(mArm.getArmState() == Arm.ArmSystemState.SHOOTING)
    {
      this.offset += offset;
      System.out.println("Offset by "+offset+" (now = "+this.offset+")");
    }
  }
  public void setOffset(double offset) {
      this.offset = offset;
  }
  public void resetOffset(){
    setOffset(0);
  }

  private double shootingArmPose(double botToSpeakerDistanceMeter)
  {
    return mShooterRPMTreeMap.getInterpolated(new InterpolatingDouble(botToSpeakerDistanceMeter)).value;
  }
}
