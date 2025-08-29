package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends Command {
    private final Shooter shooter;
    
    public ShootCommand(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {}

    
}
