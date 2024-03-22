// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.autos.autocommands;

import java.util.function.BooleanSupplier;

import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitCommand extends Command {
  private final Command executedCommand;
  private final double durationTime;
  private double startTime;
  private final BooleanSupplier mStop;
  /** Creates a new WaitCommand. 
   * <p>Executes a command for a specific period of time or stop when {@code orStop} is true</p>
   * @param command The command to execute
   * @param duration The minimum time to execute
   * @param orStop The option to force it stop.
  */
  public WaitCommand(Command command, double duration, BooleanSupplier orStop) {
    executedCommand = command;
    durationTime = duration;
    mStop = orStop;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /** 
   * Executes a command for a specific period of time
   * @see #WaitCommand(Command, double, BooleanSupplier)
  */
  public WaitCommand(Command command, double duration) {
    this(command, duration, ()-> false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    executedCommand.schedule();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    executedCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > durationTime || mStop.getAsBoolean();
  }
}
