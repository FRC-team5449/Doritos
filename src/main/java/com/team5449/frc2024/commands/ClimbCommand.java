// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.commands;

import com.team5449.frc2024.subsystems.score.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {
  private final Climber mClimb;
  private double speed;
  public ClimbCommand(Climber C,double s) {
    mClimb=C;
    speed=s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mClimb.setClimbOpenloop(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimb.setClimbOpenloop(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
