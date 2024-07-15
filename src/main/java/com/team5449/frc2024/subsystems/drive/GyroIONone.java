package com.team5449.frc2024.subsystems.drive;

public class GyroIONone implements GyroIO{

    public GyroIONone() {
        
    }
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = false;

        inputs.yaw = 0;
        inputs.pitch = 0;
        inputs.roll = 0;
        inputs.angularVelocity = 0;
        inputs.AccumGyroX = 0;
        inputs.AccumGyroY = 0;
        inputs.AccumGyroZ = 0;
    }
}
