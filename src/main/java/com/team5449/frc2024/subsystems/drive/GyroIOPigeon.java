package com.team5449.frc2024.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team5449.frc2024.Constants;
import com.team5449.frc2024.Constants.Ports;
import com.team5449.lib.CConsole;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroIOPigeon implements GyroIO{
    private double preAccumGyroZ = 0;

    private final Pigeon2 gyro;
    private StatusSignal<Double> yawSignal;
    private StatusSignal<Double> angularVelocitySignal;
    private BaseStatusSignal[] signals;

    public GyroIOPigeon() {
        gyro = new Pigeon2(Ports.kPigeonId, Ports.kCANBusFDName);

        Pigeon2Configuration config = new Pigeon2Configuration();
        MountPoseConfigs mountPoseConfigs = new MountPoseConfigs();
        mountPoseConfigs.MountPoseYaw = Constants.pigeonMountPoseYaw;
        mountPoseConfigs.MountPoseRoll = Constants.pigeonMountPoseRoll;
        mountPoseConfigs.MountPosePitch = Constants.pigeonMountPosePitch;

        GyroTrimConfigs gyroTrimConfigs = new GyroTrimConfigs();
        gyroTrimConfigs.GyroScalarZ = Constants.pigeonError;
        config.MountPose = mountPoseConfigs;
        config.GyroTrim = gyroTrimConfigs;
        gyro.getConfigurator().apply(config);

        yawSignal = gyro.getYaw();
        angularVelocitySignal = gyro.getAngularVelocityZDevice();
        signals = new BaseStatusSignal[2];
        signals[0] = yawSignal;
        signals[1] = angularVelocitySignal;
    }
    public void setZero(){
        gyro.setYaw(0);
        preAccumGyroZ = gyro.getAccumGyroZ().getValueAsDouble();
    }

    public void putYawAndAccumGyroZToDashBoard(){
        SmartDashboard.putNumber("Yaw", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("AccumGyroZ", gyro.getAccumGyroZ().getValueAsDouble()-preAccumGyroZ);
    }
    public double getAccumGyroZ(){
        return gyro.getAccumGyroZ().getValueAsDouble()-preAccumGyroZ;
    }
    public double getYaw(){
        return gyro.getYaw().getValueAsDouble();
    }
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;

        inputs.yaw = Units.degreesToRadians(
                BaseStatusSignal.getLatencyCompensatedValue(yawSignal, angularVelocitySignal));
        inputs.pitch = Units.degreesToRadians(gyro.getPitch().getValue());
        inputs.roll = Units.degreesToRadians(gyro.getRoll().getValue());
        inputs.angularVelocity = Units.degreesToRadians(angularVelocitySignal.getValue());
        inputs.AccumGyroX = gyro.getAccumGyroX().getValue();
        inputs.AccumGyroY = gyro.getAccumGyroY().getValue();
        inputs.AccumGyroZ = getAccumGyroZ();
        //CConsole.stdout.log("AccumGyroZ:",inputs.AccumGyroZ);
    }

    @Override
    public BaseStatusSignal[] getSignals() {
        return signals;
    }
}
