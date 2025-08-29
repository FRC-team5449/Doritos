package frc.robot.subsystems.arm;

import org.ejml.dense.row.decompose.UtilDecompositons_CDRM;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.arm.ArmConstants;

public class Arm extends SubsystemBase {
    private final TalonFX leftArmSlave;
    private final TalonFX rightArmSlave;
    private final CANcoder armCanCoder;

    private double setPoint = 0;

    private TalonFXConfiguration config = new TalonFXConfiguration();
    private MotionMagicDutyCycle motionMagicDutyCycle = new MotionMagicDutyCycle(0);
    public static CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    public Arm() {
        leftArmSlave = new TalonFX(ArmConstants.leftArmSlaveCanId, ArmConstants.canBusName);
        rightArmSlave = new TalonFX(ArmConstants.rightArmSlaveCanId, ArmConstants.canBusName);
        armCanCoder = new CANcoder(ArmConstants.armCanCoderCanId, ArmConstants.canBusName);

        config = ArmConstants.getConfigs();
        encoderConfig = ArmConstants.getCancoderConfig();

        rightArmSlave.getConfigurator().apply(config);
        armCanCoder.getConfigurator().apply(encoderConfig);
    }

    public void setArmPosition(double position) {
        position = Math.max(Math.min(ArmConstants.maxArmPosition, position), ArmConstants.minArmPosition);
        position -= ArmConstants.ManualOffset;
        motionMagicDutyCycle = motionMagicDutyCycle.withSlot(0);
        setPoint = position;
    }
    
    @Override
    public void periodic() {
        rightArmSlave.setControl(motionMagicDutyCycle.withPosition(setPoint));
        leftArmSlave.setControl(new Follower(ArmConstants.rightArmSlaveCanId, true));
    }
}
