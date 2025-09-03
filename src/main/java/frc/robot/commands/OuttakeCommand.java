package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;

public class OuttakeCommand extends Command {
    private final Arm arm;
    private final Shooter shooter;

    public OuttakeCommand(Arm arm, Shooter shooter) {
        this.arm = arm;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(arm.isArmReady()) {
            shooter.setShooterRPM(-25);
            shooter.setTransitSpeed(-0.4);
        }
        else {
            shooter.setShooterRPM(0);
            shooter.setTransitSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterRPM(0);
        shooter.setTransitSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}