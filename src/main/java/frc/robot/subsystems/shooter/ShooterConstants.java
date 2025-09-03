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

    public static final double kV0 = 0.16;
    public static final double kP0 = 0.11;
    public static final double kI0 = 0.0;
    public static final double kD0 = 0.0;

    public static final double timeElasp = 0.02;

    public static TalonFXConfiguration getConfigs() {
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Slot0.kV = kV0;
        config.Slot0.kP = kP0;
        config.Slot0.kI = kI0;
        config.Slot0.kD = kD0;

        return config;
    }

    public static CANcoderConfiguration getCancoderConfig() {
        mEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        return mEncoderConfig;
    }

}
