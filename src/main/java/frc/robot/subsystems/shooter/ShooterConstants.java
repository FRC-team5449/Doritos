package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ShooterConstants {
    public static final String CanBusName = "canivore";

    public static final int upShooterCanId = 40;
    public static final int lowShooterCanId = 41;
    public static final int transitCanId = 8;

    public static TalonFXConfiguration config = new TalonFXConfiguration();
    public static CANcoderConfiguration mEncoderConfig = new CANcoderConfiguration();


    public static final boolean upShooterInversed = false;
    public static final boolean lowShooterInversed = true;
    public static final boolean tansitInversed = true;

    public static TalonFXConfiguration getConfigs() {
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotionMagic.MotionMagicCruiseVelocity = 0.75;
        config.MotionMagic.MotionMagicAcceleration = 7.5;
        config.Slot0.kP = 0.03;
        config.Slot0.kV = 0.11;
        config.Feedback.FeedbackRemoteSensorID = 12;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRotorOffset = -0.57763671875;
        config.Feedback.RotorToSensorRatio = 192 * 0.75;

        return config;
    }

    public static CANcoderConfiguration getCancoderConfig() {
        mEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        return mEncoderConfig;
    }

}
