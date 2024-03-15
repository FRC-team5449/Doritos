package com.team5449.frc2024.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5449.frc2024.Constants;

import edu.wpi.first.math.util.Units;

public class SwerveModuleIOFalconPro implements SwerveModuleIO{
    private final TalonFX primaryDriveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;
    
    private double targetVelocityMetersPerSeconds = 0;
    private double targetSteerAngleRadians = 0;

    private boolean isVoltage = false;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0,0, true, 0, 0, false, false, false);
    private final VoltageOut voltageControl = new VoltageOut(0, true, false, false, false);
    private final PositionDutyCycle steerPositionControl = new PositionDutyCycle(0, 0, true, 0, 0, false, false, false);
    private final PositionVoltage steerPositionVoltage = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    private StatusSignal<Double> primaryDrivePositionSignal;
    private StatusSignal<Double> primaryDriveVelocitySignal;
    private StatusSignal<Double> steerPositionSignal;
    private StatusSignal<Double> steerVelocitySignal;
    private BaseStatusSignal[] signals;

    public SwerveModuleIOFalconPro(
            int driveMotorId,
            int steerMotorId,
            int steerEncoderId,
            String canBus,
            double steerAngleOffsetRotation,
            boolean isInverted) {
        
        primaryDriveMotor = new TalonFX(driveMotorId, canBus);
        steerMotor = new TalonFX(steerMotorId, canBus);
        steerEncoder = new CANcoder(steerEncoderId, canBus);

        FeedbackConfigs feedBackConfigs = new FeedbackConfigs();
        feedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        CurrentLimitsConfigs driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
        driveCurrentLimitsConfigs.SupplyCurrentLimit = Constants.driveCurrentLimit;
        driveCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Feedback = feedBackConfigs;
        driveConfig.CurrentLimits = driveCurrentLimitsConfigs;
        driveConfig.MotorOutput = motorOutputConfigs;
        driveConfig.Slot0.kP = Constants.kMk4DriveVelocityKp;
        driveConfig.Slot0.kI = Constants.kMk4DriveVelocityKi;
        driveConfig.Slot0.kD = Constants.kMk4DriveVelocityKd;
        driveConfig.Slot0.kS = Constants.kMk4DriveVelocityKs;
        driveConfig.Slot0.kV = Constants.kMk4DriveVelocityKv;

        primaryDriveMotor.getConfigurator().apply(driveConfig);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        FeedbackConfigs feedBackSteerConfigs = new FeedbackConfigs();
        feedBackSteerConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedBackSteerConfigs.FeedbackRemoteSensorID = steerEncoderId;
        feedBackSteerConfigs.RotorToSensorRatio = 1.0 / Constants.steerGearRatio;

        CurrentLimitsConfigs steerCurrentLimitsConfigs = new CurrentLimitsConfigs();
        steerCurrentLimitsConfigs.SupplyCurrentLimit = Constants.steerCurrentLimit;
        steerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.turnkP;
        slot0Configs.kI = Constants.turnkI;
        slot0Configs.kD = Constants.turnkD;

        steerConfig.Feedback = feedBackSteerConfigs;
        steerConfig.CurrentLimits = steerCurrentLimitsConfigs;
        steerConfig.Slot0 = slot0Configs;

        MotorOutputConfigs steerMotorOutputConfigs = new MotorOutputConfigs();
        steerMotorOutputConfigs.Inverted =  isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        steerConfig.MotorOutput = steerMotorOutputConfigs;

        steerMotor.getConfigurator().apply(steerConfig);

        CANcoderConfiguration steerEncoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs encoderMagnetSensorConfigs = new MagnetSensorConfigs();
        encoderMagnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderMagnetSensorConfigs.MagnetOffset = steerAngleOffsetRotation;
        steerEncoderConfig.MagnetSensor = encoderMagnetSensorConfigs;

        steerEncoder.getConfigurator().apply(steerEncoderConfig);


        primaryDrivePositionSignal = primaryDriveMotor.getPosition();
        primaryDriveVelocitySignal = primaryDriveMotor.getVelocity();
        steerPositionSignal = steerEncoder.getPosition();
        steerVelocitySignal = steerEncoder.getVelocity();

        signals = new BaseStatusSignal[4];

        signals[0] = primaryDrivePositionSignal;
        signals[1] = primaryDriveVelocitySignal;
        signals[2] = steerPositionSignal;
        signals[3] = steerVelocitySignal;
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        double driveAppliedVolts = primaryDriveMotor.getSupplyVoltage().getValue();
        double steerAppliedVolts = steerMotor.getSupplyVoltage().getValue();

        inputs.drivePositionMeters =
                BaseStatusSignal.getLatencyCompensatedValue(primaryDrivePositionSignal, primaryDriveVelocitySignal)
                        * (Constants.drivePositionCoefficient);

        inputs.driveVelocityMetersPerSec = primaryDriveVelocitySignal.getValue()
                * (Constants.driveVelocityCoefficient);
                
        inputs.driveCurrentDrawAmps = primaryDriveMotor.getSupplyCurrent().getValue();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.targetDriveVelocityMetersPerSec = targetVelocityMetersPerSeconds;

        inputs.steerPositionRad = Units.rotationsToRadians(
                BaseStatusSignal.getLatencyCompensatedValue(steerPositionSignal, steerVelocitySignal));
        inputs.steerPositionDeg = Math.toDegrees(inputs.steerPositionRad);
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocitySignal.getValue());
        inputs.steerCurrentDrawAmps = steerMotor.getSupplyCurrent().getValue();
        inputs.steerAppliedVolts = steerAppliedVolts;
        inputs.targetSteerPositionRad = targetSteerAngleRadians;

        inputs.steerAbsolutePositionRad =
                Units.rotationsToRadians(steerEncoder.getAbsolutePosition().getValue());
    }

    @Override
    public void setTargetSteerPosition(double targetSteerPositionRad) {
        steerMotor.setControl(isVoltage ? steerPositionVoltage : steerPositionControl
                .withPosition(Units.radiansToRotations(targetSteerPositionRad))
                .withFeedForward(0));
        this.targetSteerAngleRadians = targetSteerPositionRad;
    }

    @Override
    public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
        // primaryDriveMotor.setControl(
        //         voltageControl.withOutput((targetDriveVelocityMetersPerSec / getMaxVelocity()) * 12));
        primaryDriveMotor.setControl(
                velocityControl.withVelocity(targetDriveVelocityMetersPerSec / Constants.driveVelocityCoefficient).withSlot(0));

        Logger.recordOutput("Voltage", (targetDriveVelocityMetersPerSec / getMaxVelocity()) * 12);

        this.targetVelocityMetersPerSeconds = targetDriveVelocityMetersPerSec;
    }

    @Override
    public void resetToAbsoluteAngle() {}

    @Override
    public double getMaxVelocity() {
        return Constants.maxVelocityMeterPerSec;
    }

    @Override
    public BaseStatusSignal[] getSignals() {
        return signals;
    }
}
