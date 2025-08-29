package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ArmConstants {
    public static String canBusName = "canivore";

    public static int leftArmSlaveCanId = 14;
    public static int rightArmSlaveCanId = 13;
    public static int armCanCoderCanId = 12;

    public static TalonFXConfiguration config = new TalonFXConfiguration();
    public static CANcoderConfiguration mEncoderConfig = new CANcoderConfiguration();

    public static TalonFXConfiguration getConfigs() {
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotionMagic.MotionMagicCruiseVelocity = 0.75;
        config.MotionMagic.MotionMagicAcceleration = 7.5;
        config.Slot0.kP = 6;
        config.Slot0.kG = 0.028;
        config.Slot0.kS = 0.025390625;
        config.Slot1.kP = 1.5;
        config.Slot2.kP = 7.5;
        config.Feedback.FeedbackRemoteSensorID = 12;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRotorOffset = -0.57763671875;
        config.Feedback.RotorToSensorRatio = 192 * 0.75;

        return config;
    }

    public static CANcoderConfiguration getCancoderConfig() {
        mEncoderConfig.MagnetSensor.MagnetOffset = 0.6083984375;
        mEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        return mEncoderConfig;
    }
}
