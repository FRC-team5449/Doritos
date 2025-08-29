package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends Command{
    private final Arm arm;
    private final Shooter shooter;

    public ShootCommand(Arm arm, Shooter shooter) {
        this.arm = arm;
        this.shooter = shooter;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.setArmPosition(0.2); //TODO:fill the position
        shooter.setShooterOpenloop(0.6);
    }
}                                                                                               
