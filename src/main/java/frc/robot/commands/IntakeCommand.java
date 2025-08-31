package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeCommand extends Command {
    private final Arm arm;
    private final Intake intake;
    private final Shooter shooter;

    public IntakeCommand(Arm arm, Intake intake, Shooter shooter) {
        this.arm = arm;
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(arm.isArmReady()) {
            intake.setIntakeSpeed(0.6);
            shooter.setShooterRPM(25);
            shooter.setTransitSpeed(0.4);
        }
        else {
            intake.setIntakeSpeed(0);
            shooter.setShooterRPM(0);
            shooter.setTransitSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
        shooter.setShooterRPM(0);
        shooter.setTransitSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}