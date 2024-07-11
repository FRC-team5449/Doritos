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
import com.team5449.frc2024.Constants.Ports;
import com.team5449.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX mUpShooter;
  private final TalonFX mLowShooter;
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  private final StatusSignal<Double> mUpShooterVelocity;
  private final StatusSignal<Double> mLowShooterVelocity;
  private final TalonFX transit;
  private double upShooterSetpoint;
  private double lowShooterSetpoint;

  public Shooter(){

    mUpShooter = new TalonFX(Ports.kShooterUpId, Ports.kCANBusFDName);
    mLowShooter = new TalonFX(Ports.kShooterLowId, Ports.kCANBusFDName);
    mUpShooterVelocity = mUpShooter.getVelocity();
    mLowShooterVelocity = mLowShooter.getVelocity();

    upShooterSetpoint = 0;
    lowShooterSetpoint = 0;

    transit = new TalonFX(Ports.kTransId, Ports.kCANBusFDName);
    configureTalons();
  }

  private void configureTalons(){
    TalonFXConfiguration mConfiguration = new TalonFXConfiguration();
    mConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    mConfig.kP = 0.03;
    mConfig.kV = 0.011;
    // mConfig.kI = 0.01;
    // mConfig.kV = 0.1;
    // mConfig.kS = 0.13;
    // mConfig.kD = 0.01;
    mConfiguration.Slot0 = Slot0Configs.from(mConfig);
    //mConfiguration.Slot1 = Slot1Configs.from(mConfig);
    //mConfiguration.Slot1.kS = 0.16;
    mLowShooter.getConfigurator().apply(mConfiguration);
    mUpShooter.getConfigurator().apply(mConfiguration);
  }

  public void setShootRPM(double speed){
    upShooterSetpoint = speed*0.8;
    lowShooterSetpoint = -speed;

    updateSetpoint();
  }

  public void setOpenLoop(double percent, boolean isDifferent){
    mUpShooter.set(percent);
    mLowShooter.setControl(new Follower(Ports.kShooterUpId, isDifferent));
  }

  public void setAmpShooting(double speed){
    upShooterSetpoint = speed;
    updateSetpoint();
    lowShooterSetpoint = 0;
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
    mUpShooter.setControl(velocityControl.withVelocity(upShooterSetpoint).withSlot(0));
    mLowShooter.setControl(velocityControl.withVelocity(lowShooterSetpoint).withSlot(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("upShooterSpeed", mUpShooterVelocity.asSupplier().get());
    SmartDashboard.putNumber("lowShooterSpeed", mLowShooterVelocity.asSupplier().get());
  }
}
