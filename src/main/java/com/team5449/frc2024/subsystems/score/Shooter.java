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
import com.team5449.frc2024.RobotContainer;
import com.team5449.frc2024.Constants.Ports;
import com.team5449.lib.util.Util;

import edu.wpi.first.wpilibj.DigitalInput;
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
    mConfiguration.Slot0.kP = 5;
    mConfiguration.Slot0.kI = 0.1;
    mConfiguration.Slot0.kD = 0.01;
    mLowShooter.getConfigurator().apply(mConfiguration);
    mUpShooter.getConfigurator().apply(mConfiguration);
  }

  public void setShootRPM(double speed){
    upShooterSetpoint = speed;
    lowShooterSetpoint = -speed*0.6;

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
    SmartDashboard.putNumber("lowShooterSetpoint", lowShooterSetpoint);
    SmartDashboard.putNumber("upShooterSetpoint", upShooterSetpoint);
    mUpShooter.setControl(velocityControl.withVelocity(upShooterSetpoint));
    mLowShooter.setControl(velocityControl.withVelocity(lowShooterSetpoint));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("upShooterSpeed", mUpShooterVelocity.asSupplier().get());
    SmartDashboard.putNumber("lowShooterSpeed", mLowShooterVelocity.asSupplier().get());
  }
}
