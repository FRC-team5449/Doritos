// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.subsystems.score;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team5449.frc2024.Constants;
import com.team5449.frc2024.Constants.Ports;
import com.team5449.lib.util.Util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final TalonFX mArmMaster;
  private final TalonFX mArmSlave;
  private final CANcoder mArmCancoder;
  private MotionMagicDutyCycle motionMagicDutyCycle = new MotionMagicDutyCycle(0, true, 0, 0, false, false, false);
  //private DynamicMotionMagicDutyCycle dynamicMotion = new DynamicMotionMagicDutyCycle(0, 0, 0, 0, isArmAtSetpoint(), 0, 0, isArmAtSetpoint(), isArmAtSetpoint(), isArmAtSetpoint())
  private StatusSignal<Double> armPosition;
  private double setPoint;
  private static final double ManualOffset = 0.708740234375-0.6083984375;
  private static final Arm mArm = new Arm();

  private ArmSystemState mSystemState = ArmSystemState.IDLE;
  private final ShuffleboardTab mTab = Shuffleboard.getTab("Arm");
  private final GenericEntry mPos = mTab.add("ArmPosition", mSystemState.armPose).getEntry();
  private final GenericEntry mStateName = mTab.add("ArmSystemState", mSystemState.toString()).getEntry();

  //private static int ArmSystemState_LoadCount = 0;

  public enum ArmSystemState{
    IDLE(0.03),
    SHOOTING(0.118),
    OVERSHOOT(0.21),
    AMP(0.38),
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

  public ArmSystemState getArmState()
  {
    if(mArm.isArmAtSetpoint())
    {
      return mSystemState;
    }else{
      return ArmSystemState.CHANGING;
    }
  }

  public void setPose(ArmSystemState newSystemState){
    mSystemState = newSystemState;
    //CConsole.stdout.log("Setted new SystemState", mSystemState);
    mStateName.setString(mSystemState.toString());
    OnArmPoseUpdate();
  }
  private void OnArmPoseUpdate()
  {
    mPos.setDouble(mSystemState.armPose);
  }
  public void setAutoShootPosition(double position){
    setPose(ArmSystemState.AUTOSHOOT);
    mSystemState.armPose = position;
    OnArmPoseUpdate();
  }

  private Arm() {
    mArmMaster = new TalonFX(Ports.kArmMasterId,Ports.kCANBusFDName);
    mArmSlave = new TalonFX(Ports.kArmSlaveId,Ports.kCANBusFDName);
    mArmCancoder = new CANcoder(12, Ports.kCANBusFDName);
    mArmMaster.setNeutralMode(NeutralModeValue.Brake);
    mArmSlave.setNeutralMode(NeutralModeValue.Brake);
    armPosition = mArmMaster.getPosition();
    armPosition.setUpdateFrequency(250);
    setArmPosition(0.03);
    configureTalons();
  }

  private void configureTalons(){
    TalonFXConfiguration mConfig = new TalonFXConfiguration();
    mConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    mConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    mConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    mConfig.MotionMagic.MotionMagicCruiseVelocity = 0.75;
    mConfig.MotionMagic.MotionMagicAcceleration = 7.5;
    mConfig.Slot0.kP = 6;
    mConfig.Slot0.kG = 0.028;
    mConfig.Slot0.kS = 0.025390625;

    mConfig.Slot1.kP = 1.5;

    mConfig.Slot2.kP = 7.5;

    mConfig.Feedback.FeedbackRemoteSensorID = 12;
    mConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    mConfig.Feedback.FeedbackRotorOffset = -0.57763671875;
    mConfig.Feedback.RotorToSensorRatio = 192 * 0.75;

    CANcoderConfiguration mEncoderConfig = new CANcoderConfiguration();
    mEncoderConfig.MagnetSensor.MagnetOffset = 0.6083984375;
    mEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;


    mArmMaster.getConfigurator().apply(mConfig);
    mArmCancoder.getConfigurator().apply(mEncoderConfig);
    
  }

  public static Arm getInstance(){
    return mArm;
  }

  public void setTargetOpenLoop(double percent){
    mArmMaster.setControl(new DutyCycleOut(percent));
  }

  private void setArmPositionCommon(double position){
    position = Util.limit(Constants.maxArmPosition, Constants.minArmPosition, position);
    position -= ManualOffset;
    setPoint = position;
  }

  public void setArmPosition(double position){
    setArmPositionCommon(position);
    motionMagicDutyCycle = motionMagicDutyCycle.withSlot(0);
    SmartDashboard.putNumber("Arm/Setpoint(Rot)", setPoint+ManualOffset);
  }

  public boolean isArmAtSetpoint(){
    return Util.epsilonEquals(setPoint, armPosition.asSupplier().get(), 0.01);
  }

  public void setArmClimbPosition(double position){
    setArmPositionCommon(position);
    motionMagicDutyCycle = motionMagicDutyCycle.withSlot(1);
  }

  public void setAutoArmDown(double position){
    setArmPositionCommon(position);
    motionMagicDutyCycle = motionMagicDutyCycle.withSlot(2);
  }

  @Override
  public void periodic() {
    mArmMaster.setControl(motionMagicDutyCycle.withPosition(setPoint));
    mArmSlave.setControl(new Follower(Ports.kArmMasterId, true));

    SmartDashboard.putNumber("Arm/Position", armPosition.asSupplier().get()+ManualOffset);


    


  if(mSystemState == ArmSystemState.PRETRAP){
    setArmClimbPosition(mSystemState.armPose);
  }
  else if(mSystemState == ArmSystemState.ARMDOWN){
    setAutoArmDown(mSystemState.armPose);
  }
  else{
    setArmPosition(mSystemState.armPose);
  }

  //SmartDashboard.putNumber("Arm Pose", mSystemState.armPose);
  }

  public void setShootArmPose(double pose){
    ArmSystemState.SHOOTING.armPose = pose;
    
    OnArmPoseUpdate();
  }
}
