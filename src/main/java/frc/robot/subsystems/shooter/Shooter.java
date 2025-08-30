package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Shooter extends SubsystemBase{
    private final TalonFX upShooter;
    private final TalonFX lowShooter;
    private final TalonFX transit;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);
    @AutoLogOutput(key = "RobotState")
    private final StatusSignal<AngularVelocity> upShooterVelocity;
    @AutoLogOutput(key = "RobotState")
    private final StatusSignal<AngularVelocity> lowShooterVelocity;
    private double upShooterSetpoint;
    private double lowShooterSetpoint;
    

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
    }


    public void setTransitSpeed(double percent) {
        transit.set(percent);
    }

    public void setShooterRPM(double speed) {
        upShooterSetpoint = speed;
        lowShooterSetpoint = -speed;
    }

    public boolean isShooterAtSetpoint(){
        return Util.epsilonEquals(upShooterSetpoint, upShooter.getVelocity().getValue().in(RPM),7);
      }

    @Override
    public void periodic() {
        //System.out.println(upShooterVelocity.getValue().in(RotationsPerSecond));
        //System.out.println(upShooter.getVelocity().getValue().in(RPM));
        System.out.println(isShooterAtSetpoint());
        upShooter.setControl(velocityControl.withVelocity(upShooterSetpoint).withSlot(0));
        lowShooter.setControl(new Follower(ShooterConstants.upShooterCanId, true));
    }
}
