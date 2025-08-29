package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Shooter extends SubsystemBase{
    private final TalonFX upShooter;
    private final TalonFX lowShooter;
    private final TalonFX transit;
    private final StatusSignal<AngularVelocity> upShooterVelocity;
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
    }

    public void setShooterOpenloop(double percent) {
        upShooter.set(percent);
        lowShooter.setControl(new Follower(ShooterConstants.upShooterCanId, false));
    }

    public void setTransitSpeed(double percent) {
        transit.set(percent);
    }

    public void setShooterRPM(double speed) {
        upShooterSetpoint = speed;
        lowShooterSetpoint = -speed;
    }

    public boolean isShooterAtSetpoint(){
        return Util.epsilonEquals(upShooterSetpoint, upShooterVelocity.getValueAsDouble(), 5) && Util.epsilonEquals(lowShooterSetpoint, lowShooterVelocity.getValueAsDouble(), 5);
      }

    @Override
    public void periodic() {
    }
}
