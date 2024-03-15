// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.commands;

import com.team5449.frc2024.subsystems.vision.LedSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class LedCommand extends Command {
  private LedSubsystem mLedSubsystem;
  private double mStartTime;
  private double mDuration;
  private Color mColor;
  
  public LedCommand(Color color,double duration) {
    mLedSubsystem=new LedSubsystem();
    mStartTime=Timer.getFPGATimestamp();
    mDuration=duration;
    mColor=color;
  }
  public LedCommand(Color color) {
    mLedSubsystem=new LedSubsystem();
    mStartTime=Timer.getFPGATimestamp();
    mDuration=0;
    mColor=color;
  }
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(Timer.getFPGATimestamp()-mStartTime<mDuration){
      mLedSubsystem.setPureColor(mColor);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mLedSubsystem.setPureColor(Color.kWhite);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
