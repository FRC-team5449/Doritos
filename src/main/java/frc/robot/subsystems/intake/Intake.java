package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private final TalonFX upIntake;
    private final TalonFX downIntake;

    public Intake() {
        upIntake = new TalonFX(Constants.Ports.kIntake1Id, Constants.Ports.kCANBusFDName);
        downIntake = new TalonFX(Constants.Ports.kIntake2Id, Constants.Ports.kCANBusFDName);
        upIntake.setInverted(Constants.upShooterInversed);
        downIntake.setInverted(Constants.lowShooterInversed);
    }

    public void setIntakeSpeed(double percent) {
        upIntake.set(percent);
        downIntake.set(percent);
    }

    @Override
    public void periodic() {
    }
}
