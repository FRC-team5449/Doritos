// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.subsystems.score;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team5449.frc2024.Constants.Ports;
import com.team5449.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX mUpShooter;
  private final TalonFX mLowShooter;
  private final VelocityDutyCycle velocityControl = new VelocityDutyCycle(0);
  private final StatusSignal<Double> mUpShooterVelocity;
  private final StatusSignal<Double> mLowShooterVelocity;
  private final TalonFX transit;
  private double upShooterSetpoint;
  private double lowShooterSetpoint;
  private static final Shooter mInstance = new Shooter();

  private Shooter(){

    mUpShooter = new TalonFX(Ports.kShooterUpId, Ports.kCANBusFDName);
    mLowShooter = new TalonFX(Ports.kShooterLowId, Ports.kCANBusFDName);
    mUpShooterVelocity = mUpShooter.getVelocity();
    mLowShooterVelocity = mLowShooter.getVelocity();

    upShooterSetpoint = 0;
    lowShooterSetpoint = 0;

    transit = new TalonFX(Ports.kTransId, Ports.kCANBusFDName);
    configureTalons();
  }
  public static Shooter getInstance(){
    return mInstance;
  }

  private void configureTalons(){
    TalonFXConfiguration mConfiguration = new TalonFXConfiguration();
    mConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    mConfiguration.Slot0.kP = 0.01;
    mConfiguration.Slot0.kI = 0.01;
    mConfiguration.Slot0.kV = 0.001;
    mConfiguration.Slot0.kS = 0.13;
    mLowShooter.getConfigurator().apply(mConfiguration);
    mConfiguration.Slot0.kS = 0.16;
    mUpShooter.getConfigurator().apply(mConfiguration);
  }

  public void setShootRPM(double speed){
    upShooterSetpoint = speed;
    lowShooterSetpoint = -speed;

    updateSetpoint();
  }

  public void setOpenLoop(double percent, boolean isDifferent){
    mUpShooter.set(percent);
    mLowShooter.set(isDifferent?-percent:percent);
    //mLowShooter.setControl(new Follower(Ports.kShooterUpId, isDifferent));
  }

  public void setAmpShooting(double speed){
    upShooterSetpoint = speed;
    lowShooterSetpoint = 0;
    updateSetpoint();
    mLowShooter.set(0);
    //mLowShooter.setControl(new Follower(Ports.kShooterUpId, false));
  }

  public boolean isShooterAtSetpoint(){
    return Util.epsilonEquals(upShooterSetpoint, mUpShooterVelocity.asSupplier().get(), 5) && Util.epsilonEquals(lowShooterSetpoint, mLowShooterVelocity.asSupplier().get(), 5);
  }

  public void transit(double percent){
    transit.set(percent);
  }
  
  private void updateSetpoint()
  {
    SmartDashboard.putNumber("Shooter/lowSetpoint", lowShooterSetpoint);
    SmartDashboard.putNumber("Shooter/upSetpoint", upShooterSetpoint);
    mUpShooter.setControl(velocityControl.withVelocity(upShooterSetpoint));
    mLowShooter.setControl(velocityControl.withVelocity(lowShooterSetpoint));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/upSpeed", mUpShooterVelocity.asSupplier().get());
    SmartDashboard.putNumber("Shooter/lowSpeed", mLowShooterVelocity.asSupplier().get());
  }
}
