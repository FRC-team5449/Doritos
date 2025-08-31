package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Shooter extends SubsystemBase{
    private final TalonFX upShooter;
    private final TalonFX lowShooter;
    private final TalonFX transit;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private final PIDController velocityPID;
    @AutoLogOutput(key = "RobotState")
    private final StatusSignal<AngularVelocity> upShooterVelocity;
    @AutoLogOutput(key = "RobotState")
    private final StatusSignal<AngularVelocity> lowShooterVelocity;
    private double upShooterSetpoint;
    private double lowShooterSetpoint;
    private double currentVelocity;
    private double currentPosition;
    private double lastPosition;
    @AutoLogOutput(key = "RobotState")
    private double output;
    private double feedforward;
    

    public Shooter() {
        upShooter = new TalonFX(ShooterConstants.upShooterCanId, ShooterConstants.CanBusName);
        lowShooter = new TalonFX(ShooterConstants.lowShooterCanId, ShooterConstants.CanBusName);
        transit = new TalonFX(ShooterConstants.transitCanId, ShooterConstants.CanBusName);
        upShooter.setInverted(ShooterConstants.upShooterInversed);
        lowShooter.setInverted(ShooterConstants.lowShooterInversed);
        transit.setInverted(ShooterConstants.tansitInversed);
        upShooterVelocity = upShooter.getVelocity();
        lowShooterVelocity = lowShooter.getVelocity();

        config = ShooterConstants.getConfigs();

        upShooter.getConfigurator().apply(config);
        lowShooter.getConfigurator().apply(config);

        velocityPID = new PIDController(
            ShooterConstants.kP0, 
            ShooterConstants.kI0, 
            ShooterConstants.kD0
        );

    }


    public void setTransitSpeed(double percent) {
        transit.set(percent);
    }

    public void setShooterRPM(double speed) {
        upShooterSetpoint = speed;
    }

    public boolean isShooterAtSetpoint(){
        return Util.epsilonEquals(upShooterSetpoint, getShooterVelocity(),3);
      }

    public double getShooterVelocity() {
        return currentVelocity;
    }

    @Override
    public void periodic() {
        //System.out.println(upShooterVelocity.getValue().in(RotationsPerSecond));
        //System.out.println(upShooter.getVelocity().getValue().in(RPM));
        currentPosition = upShooter.getPosition().getValue().in(Rotation);
        currentVelocity = (currentPosition - lastPosition) / ShooterConstants.timeElasp;
        output = velocityPID.calculate(currentVelocity, upShooterSetpoint);
        feedforward = ShooterConstants.kV0 * upShooterSetpoint;
        
            
        upShooter.setVoltage(output + feedforward);
        lowShooter.setControl(new Follower(ShooterConstants.upShooterCanId, true));

        SmartDashboard.putNumber("Shooter/upShooterSetpoint", upShooterSetpoint);
        SmartDashboard.putNumber("Shooter/upShooterVelocity", upShooterVelocity.getValue().in(RPM));
        SmartDashboard.putNumber("Shooter/upShooterCalculatedVelocity", getShooterVelocity());
        SmartDashboard.putNumber("Output", output);

        lastPosition = currentPosition;
    }
}
