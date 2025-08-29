package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeCommand extends Command {
    private final Intake intake;
    private final Shooter shooter;

    public IntakeCommand(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intake.setIntakeSpeed(0.6);
        shooter.setShooterSpeed(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
        shooter.setShooterSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}