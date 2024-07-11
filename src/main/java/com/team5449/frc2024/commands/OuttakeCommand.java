package com.team5449.frc2024.commands;

import com.team5449.frc2024.subsystems.score.Intake;
import com.team5449.frc2024.subsystems.score.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeCommand extends Command{
private final Intake mIntake;
  private final Shooter mShooter;
  /** Creates a new IntakeCommand. */
  public OuttakeCommand(Shooter shooter, Intake intake) {
    mIntake = intake;
    mShooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.setIntakeSpeed(-1);
    mShooter.setOpenLoop(0.1, true);
    mShooter.transit(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.setIntakeSpeed(0);
    mShooter.setOpenLoop(0, false);
    mShooter.transit(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
