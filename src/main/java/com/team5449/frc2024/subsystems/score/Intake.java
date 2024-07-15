// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.subsystems.score;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team5449.frc2024.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX upIntake;
    private final TalonFX downIntake;
    private static final Intake mInstance = new Intake();
    public static Intake getInstance(){
        return mInstance;
    }
    /** Creates a new Intake. */
    private Intake() {
        upIntake=new TalonFX(Constants.Ports.kIntake1Id);
        downIntake=new TalonFX(Constants.Ports.kIntake2Id);
        upIntake.setInverted(false);
        downIntake.setInverted(false);
    }

    public void setIntakeSpeed(double percent){
        upIntake.set(percent);
        downIntake.set(percent);
    }

    @Override
    public void periodic() {
    }
}
