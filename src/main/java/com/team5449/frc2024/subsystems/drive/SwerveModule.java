package com.team5449.frc2024.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final String name;

    public SwerveModule(SwerveModuleIO io, String name) {
        this.io = io;
        this.name = name;
    }

    public void setTargetState(SwerveModuleState targetState) {
        double currentAngle = inputs.steerPositionRad;
        double targetAngle = MathUtil.inputModulus(
                targetState.angle.getRadians(),
                0,
                2 * Math.PI);

        double absoluteAngle = MathUtil.inputModulus(
                currentAngle, 0, 2 * Math.PI);

        double angleError = MathUtil.inputModulus(
                targetAngle - absoluteAngle,
                -Math.PI,
                Math.PI);

        double resultAngle = currentAngle + angleError;

        io.setTargetDriveVelocity(targetState.speedMetersPerSecond);
        io.setTargetSteerPosition(resultAngle);
    }

    public void updateInputs() {
        if (DriverStation.isDisabled()) {
            io.resetToAbsoluteAngle();
        }

        io.updateInputs(inputs);
        Logger.processInputs("Drive/" + name + "Module", inputs);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePositionMeters, new Rotation2d(inputs.steerPositionRad));
    }

    public SwerveModuleState getMeasuredState(){
        return new SwerveModuleState(inputs.driveVelocityMetersPerSec, Rotation2d.fromRadians(inputs.steerAbsolutePositionRad));
    }

    public BaseStatusSignal[] getSignals() {
        return io.getSignals();
    }
}
