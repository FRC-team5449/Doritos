// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.subsystems.score;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;
import com.revrobotics.CANSparkBase.ControlType;
import com.team5449.frc2024.Constants;
import com.team5449.frc2024.Constants.Ports;
import com.team5449.lib.util.Util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFalcon extends SubsystemBase {
  private final TalonFX mUpShooter;
  private final TalonFX mLowShooter;
  private final StatusSignal<Double> mUpShooterVelocity;
  private final StatusSignal<Double> mLowShooterVelocity;
  private final PWMTalonSRX mTransit;

  //private final DigitalInput intakeDIO;

  private boolean lastIntakeVlue;

  private boolean isIntakeInShooter;

  private VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  /** Creates a new ShooterFalcon. */
  public ShooterFalcon() {
    mUpShooter = new TalonFX(Ports.kShooterUpId, Ports.kCANBusFDName);
    mLowShooter = new TalonFX(Ports.kShooterLowId, Ports.kCANBusFDName);

    mUpShooterVelocity = mUpShooter.getVelocity();
    mLowShooterVelocity = mLowShooter.getVelocity();

    //intakeDIO = new DigitalInput(Ports.kIntakrDIOChannel);

    mTransit = new PWMTalonSRX(Ports.kTransId);

    lastIntakeVlue = false;

    configureTalon();
  }

  private void configureTalon(){
    TalonFXConfiguration shooteConfiguration = new TalonFXConfiguration();
    shooteConfiguration.Slot0.kP = 5;
    shooteConfiguration.Slot0.kI = 0.1;
    shooteConfiguration.Slot0.kD = 0.01;
    shooteConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    shooteConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = -40;
    shooteConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    mUpShooter.getConfigurator().apply(shooteConfiguration);
  }

  public void setShootRPM(double speed){
    speed = Util.limit(75, -75, speed);
    mUpShooter.setControl(velocityControl.withVelocity(speed));
    mLowShooter.setControl(new Follower(Ports.kShooterUpId, false));
  }

  public boolean isShooterAtSetpoint(double setpoint){
    return Util.epsilonEquals(setpoint, mUpShooterVelocity.asSupplier().get(), 500) &&
      Util.epsilonEquals(setpoint, mLowShooterVelocity.asSupplier().get(), 500);
  }


  public void transit(double percent){
    mTransit.set(percent);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Up Shooter Speed", mUpShooterVelocity.asSupplier().get());
    SmartDashboard.putNumber("Low Shooter Speed", mLowShooterVelocity.asSupplier().get());

    // if(!lastIntakeVlue && intakeDIO.get()){
    //   isIntakeInShooter = true;
    //   lastIntakeVlue = intakeDIO.get();
    // }


  }
}
