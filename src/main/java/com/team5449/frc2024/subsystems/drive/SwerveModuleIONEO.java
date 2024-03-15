package com.team5449.frc2024.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team5449.frc2024.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveModuleIONEO implements SwerveModuleIO{
    private final CANSparkMax primaryDriveMotor;
    private final CANSparkMax steerMotor;
    private final CANcoder steerEncoder;
    private Rotation2d mOffset;
    private final Rotation2d mEncoderZero;

    private double targetVelocityMetersPerSeconds = 0;
    private double targetSteerAngleRadians = 0;

    public SwerveModuleIONEO(        
        int driveMotorId,
        int steerMotorId,
        int steerEncoderId,
        double steerAngleOffsetRotation){
        
        primaryDriveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorId, MotorType.kBrushless);
        steerEncoder = new CANcoder(steerEncoderId);
        mEncoderZero = Rotation2d.fromRotations(steerAngleOffsetRotation);

        primaryDriveMotor.setInverted(true);
        primaryDriveMotor.setSmartCurrentLimit(Constants.driveCurrentLimit);
        primaryDriveMotor.setIdleMode(IdleMode.kCoast);
        primaryDriveMotor.getPIDController().setP(Constants.drivekP, 0);
        primaryDriveMotor.getPIDController().setI(Constants.drivekI, 0);
        primaryDriveMotor.getPIDController().setD(Constants.drivekD, 0);
        primaryDriveMotor.getPIDController().setFF(Constants.drivekD, 0);

        steerMotor.setInverted(true);
        steerMotor.setSmartCurrentLimit(Constants.steerCurrentLimit);
        steerMotor.setIdleMode(IdleMode.kCoast);
        steerMotor.getPIDController().setP(Constants.turnkP, 0);
        steerMotor.getPIDController().setI(Constants.turnkI, 0);
        steerMotor.getPIDController().setD(Constants.turnkD, 0);
        steerMotor.getPIDController().setFF(Constants.turnkP, 0);

        CANcoderConfiguration steerEncoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs encoderMagnetSensorConfigs = new MagnetSensorConfigs();
        encoderMagnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderMagnetSensorConfigs.MagnetOffset = 0;
        steerEncoderConfig.MagnetSensor = encoderMagnetSensorConfigs;

        steerEncoder.getConfigurator().apply(steerEncoderConfig);

        rezeroSteeringMotor();
    }

    public void rezeroSteeringMotor(){
        mOffset = Rotation2d.fromRadians(steerMotor.getEncoder().getPosition() * Constants.steerPositionCoefficient).rotateBy(getAdjustedSteerEncoderAngle().unaryMinus());
    }

    public Rotation2d getSteerEncoderAngle(){
        return Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().refresh().getValue());
    }

    public Rotation2d getAdjustedSteerEncoderAngle() {
        return getSteerEncoderAngle().rotateBy(mEncoderZero.unaryMinus());
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs){
        inputs.drivePositionMeters = primaryDriveMotor.getEncoder().getPosition();
        inputs.driveVelocityMetersPerSec = primaryDriveMotor.getEncoder().getVelocity() * Constants.driveVelocityCoefficient / 60;
        inputs.driveAppliedVolts = primaryDriveMotor.getBusVoltage();
        inputs.driveCurrentDrawAmps = primaryDriveMotor.getOutputCurrent();
        inputs.targetDriveVelocityMetersPerSec = targetVelocityMetersPerSeconds;

        inputs.steerPositionRad = Units.rotationsToRadians(steerMotor.getEncoder().getPosition());
        inputs.steerPositionDeg = Units.rotationsToDegrees(steerMotor.getEncoder().getPosition());
        inputs.steerVelocityRadPerSec = Units.degreesToRadians(steerMotor.getEncoder().getVelocity());
        inputs.steerAppliedVolts = steerMotor.getBusVoltage();
        inputs.driveCurrentDrawAmps = steerMotor.getOutputCurrent();
        inputs.targetSteerPositionRad = targetSteerAngleRadians;

        inputs.steerAbsolutePositionRad = Units.rotationsToRadians(steerEncoder.getAbsolutePosition().getValue());
    }

    @Override
    public void setTargetSteerPosition(double targetSteerPositionRad) {
        steerMotor.getPIDController().setReference((targetSteerPositionRad + mOffset.getRadians()) / Constants.steerPositionCoefficient, CANSparkBase.ControlType.kPosition);
        this.targetSteerAngleRadians = targetSteerPositionRad;
    }


    @Override
    public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec){
        primaryDriveMotor.getPIDController().setReference(targetDriveVelocityMetersPerSec / Constants.driveVelocityCoefficient * 60, CANSparkBase.ControlType.kVelocity);
        this.targetVelocityMetersPerSeconds = targetDriveVelocityMetersPerSec;
    }

    @Override
    public void resetToAbsoluteAngle() {}


    @Override
    public double getMaxVelocity(){
        return Constants.maxVelocityMeterPerSec;
    }

    @Override
    public BaseStatusSignal[] getSignals() {
        return new BaseStatusSignal[0];
    }
}
