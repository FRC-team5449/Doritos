package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Shooter extends SubsystemBase{
    private final TalonFX upShooter;
    private final TalonFX lowShooter;

    public Shooter() {
        upShooter = new TalonFX(ShooterConstants.upShooterCanId, ShooterConstants.CanBusName);
        lowShooter = new TalonFX(ShooterConstants.lowShooterCanId, ShooterConstants.CanBusName);
        upShooter.setInverted(ShooterConstants.upShooterInversed);
        lowShooter.setInverted(ShooterConstants.lowShooterInversed);
    }

    public void setShooterSpeed(double percent) {
        upShooter.set(percent);
        lowShooter.set(percent);
    }

    @Override
    public void periodic() {
    }
}
