package com.team5449.frc2024.commands;


import com.team5449.frc2024.Constants;
import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;
import com.team5449.frc2024.subsystems.score.Arm;
import com.team5449.frc2024.subsystems.vision.VisionSubsystem;
import com.team5449.lib.interpolate.InterpolatingDouble;
import com.team5449.lib.interpolate.InterpolatingTreeMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmPoseCommand extends Command{
private final Arm mArm;
private final DrivetrainSubsystem mDrive;
private ArmSystemState mSystemState = ArmSystemState.IDLE;
private final ShuffleboardTab mTab = Shuffleboard.getTab("Arm");
private final GenericEntry mPos = mTab.add("ArmPosition", mSystemState.armPose).getEntry();
private final GenericEntry mStateName = mTab.add("ArmSystemState", mSystemState.toString()).getEntry();
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

  public ArmPoseCommand(Arm arm, DrivetrainSubsystem drive) {
    mArm = arm;
    mDrive = drive;
    addRequirements(mArm);
  }

  public void setPose(ArmSystemState newSystemState){
    mSystemState = newSystemState;
    //CConsole.stdout.log("Setted new SystemState", mSystemState);
    mStateName.setString(mSystemState.toString());
    OnArmPoseUpdate();
  }

  private void OnArmPoseUpdate()
  {
    // mPos.setDouble(mSystemState.armPose);
    execute();
  }
  public void setAutoShootPosition(double position){
    setPose(ArmSystemState.AUTOSHOOT);
    mSystemState.armPose = position;
    OnArmPoseUpdate();
  }

  private Translation2d GetStageTranslation(){
    return (DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue?Constants.ShootTargetPosBlue:Constants.ShootTargetPosRed).toTranslation2d();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double botToTarget = mDrive.getPose().getTranslation().getDistance(GetStageTranslation());
    // SmartDashboard.putNumber("Dist", botToTarget);
    mDist.setDouble(botToTarget);

    if(mSystemState == ArmSystemState.SHOOTING){
        
        //botToTarget = -2;
      if(botToTarget < 0){
        mSystemState.armPose = 0.148;
      }
      else{
        //double armPose = Units.radiansToRotations(Math.atan((com.team5449.frc2024.Constants.armToSpeakerVerticalMeter) / Math.abs(botToTarget)));
        //mSystemState.armPose = armPose + 0.11;
        mSystemState.armPose = shootingArmPose(botToTarget);
        //CConsole.stdout.log("Setted armpose", mSystemState, "= shootingArmPose(", botToTarget, ") =", mSystemState.armPose);
      }
      mSystemState.armPose += offset;
      // OnArmPoseUpdate();

    // if(mPos.getDouble(mSystemState.armPose)!=mSystemState.armPose)
    // {
    //   mSystemState.armPose = mPos.getDouble(0);
    // }
    }


    if(mSystemState == ArmSystemState.PRETRAP){
      mArm.setArmClimbPosition(mSystemState.armPose);
    }
    else if(mSystemState == ArmSystemState.ARMDOWN){
      mArm.setAutoArmDown(mSystemState.armPose);
    }
    else{
      mArm.setArmPosition(mSystemState.armPose);
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
  
  public ArmSystemState getArmState()
  {
    if(mArm.isArmAtSetpoint())
    {
      return mSystemState;
    }else{
      return ArmSystemState.CHANGING;
    }
  }

  public void offsetBy(double offset)
  {
    if(mSystemState == ArmSystemState.SHOOTING)
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

  //private static int ArmSystemState_LoadCount = 0;

  public enum ArmSystemState{
    IDLE(0.03),
    SHOOTING(0.118),
    OVERSHOOT(0.21),
    AMP(0.36),
    INTAKE(0.02),
    OUTTAKE(0.07),
    CHANGING(0.03),
    AUTOSHOOT(0.118),
    PRECLIMB(0.22),
    CLIMB(0.085),
    PRETRAP(0.22),
    ARMDOWN(0.02),
    TRAP(0.37);
    
    public double armPose;
    public String PrintString;
    
    private ArmSystemState(double armPose){
        this.armPose = armPose;
        /*java.lang.reflect.Field[] fields = this.getClass().getDeclaredFields();
        //TODO: unpredictable bug may caused by field's order doesn't match the initalized.
        PrintString = fields[ArmSystemState_LoadCount].getName();
        ArmSystemState_LoadCount++;*/
        PrintString=this.name();
	  }

    @Override
    public String toString()
    {
        return "ArmSystemState."+PrintString+"(armPose = "+armPose+")";
    }
  }

  private double shootingArmPose(double botToSpeakerDistanceMeter)
  {
    return mShooterRPMTreeMap.getInterpolated(new InterpolatingDouble(botToSpeakerDistanceMeter)).value;
  }
}
