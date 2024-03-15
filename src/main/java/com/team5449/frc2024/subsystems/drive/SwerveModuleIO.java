package com.team5449.frc2024.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team5449.frc2024.Constants;

public interface SwerveModuleIO {

    default void updateInputs(SwerveModuleIOInputs swerveModuleIOInputs) {}

    default void setTargetSteerPosition(double targetSteerPositionRad) {}

    default void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {}

    default void resetToAbsoluteAngle() {}

    default double getMaxVelocity() {
        return Constants.maxVelocityMeterPerSec;
    }

    default BaseStatusSignal[] getSignals() {
        return new BaseStatusSignal[0];
    }


    @AutoLog
    class SwerveModuleIOInputs {
        
        public double drivePositionRad = 0;

        public double drivePositionMeters = 0;

        public double driveVelocityMetersPerSec = 0;

        public double driveCurrentDrawAmps = 0;

        public double driveAppliedVolts = 0;

        public double steerPositionTicks = 0;

        public double steerPositionRad = 0;

        public double steerPositionDeg = 0;

        public double steerVelocityRadPerSec = 0;

        public double steerCurrentDrawAmps = 0;

        public double steerAppliedVolts = 0;

        public double steerAbsolutePositionRad = 0;

        public double targetDriveVelocityMetersPerSec = 0;

        public double targetSteerPositionRad = 0;
    }
}
