// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.subsystems.score;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax mIntake1;
  private static final Intake mInstance = new Intake();
  public static Intake getInstance(){
    return mInstance;
  }
  /** Creates a new Intake. */
  private Intake() {
    mIntake1 = new CANSparkMax(6, MotorType.kBrushless);
    mIntake1.setInverted(true);
  }

  public void setIntakeSpeed(double percent){
    mIntake1.set(percent);
  }

  @Override
  public void periodic() {
  }
}
