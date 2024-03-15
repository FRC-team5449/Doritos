package com.team5449.frc2024.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;

public interface GyroIO {
    default void updateInputs(GyroIOInputs inputs) {}

    default BaseStatusSignal[] getSignals() {
        return new BaseStatusSignal[0];
    }

    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public double yaw = 0;
        public double pitch = 0;
        public double roll = 0;
        public double angularVelocity = 0;
        public double AccumGyroX = 0;
        public double AccumGyroY = 0;
        public double AccumGyroZ = 0;
    }
}
