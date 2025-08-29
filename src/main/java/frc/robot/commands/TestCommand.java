package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class TestCommand extends Command{
    private final Arm arm;

    public TestCommand(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.setArmPosition(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
