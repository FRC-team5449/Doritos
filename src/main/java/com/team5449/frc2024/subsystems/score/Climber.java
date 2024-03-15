// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.subsystems.score;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5449.frc2024.Constants.Ports;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TalonFX mClimbLft;
  public Climber() {
    mClimbLft = new TalonFX(Ports.kClimbLftId, Ports.kCANBusFDName);
    mClimbLft.setNeutralMode(NeutralModeValue.Brake);
  }
  public void setClimbOpenloop(double speed){
    mClimbLft.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
