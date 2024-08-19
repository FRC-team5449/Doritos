package com.team5449.frc2024.autos.autocommands;

import com.team5449.frc2024.commands.ArmPoseCommand;
import com.team5449.frc2024.commands.ShootCommand;
import com.team5449.frc2024.commands.ArmPoseCommand.ArmSystemState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoShootCommand extends ParallelDeadlineGroup{
    public AutoShootCommand(ShootCommand mShoot, ArmPoseCommand mArm, double timeOutSec){
        super(new WaitCommand(timeOutSec), new InstantCommand(() -> mArm.setPose(ArmSystemState.SHOOTING)).andThen(mShoot));
    }
}