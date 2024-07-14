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

  private static final Arm mInstance = new Arm();
  

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
    mConfig.MotionMagic.MotionMagicCruiseVelocity = 2;
    mConfig.MotionMagic.MotionMagicAcceleration = 7.5;
    mConfig.Slot0.kP = 10;
    mConfig.Slot0.kG = 0.028;
    mConfig.Slot0.kS = 0.025390625;

    mConfig.Slot1.kP = 1.5;

    mConfig.Slot2.kP = 7.5;

    mConfig.Feedback.FeedbackRemoteSensorID = 12;
    mConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    mConfig.Feedback.FeedbackRotorOffset = 0;//-0.57763671875;
    mConfig.Feedback.RotorToSensorRatio = 192 * 0.75;

    CANcoderConfiguration mEncoderConfig = new CANcoderConfiguration();
    mEncoderConfig.MagnetSensor.MagnetOffset = 0.6083984375;
    mEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;


    mArmMaster.getConfigurator().apply(mConfig);
    mArmCancoder.getConfigurator().apply(mEncoderConfig);
    
  }

  public static Arm getInstance(){
    return mInstance;
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
  }
}
