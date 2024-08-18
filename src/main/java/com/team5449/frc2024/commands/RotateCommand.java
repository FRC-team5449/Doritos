package com.team5449.frc2024.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.team5449.frc2024.subsystems.CommandSwerveDrivetrain;
import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;
import com.team5449.lib.CConsole;
import com.team5449.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateCommand extends Command{
    private final CommandSwerveDrivetrain drivetrain;

    private final DoubleSupplier RotateRadianAbslute;

    private ChassisSpeeds targetVelocity = new ChassisSpeeds();

    private PIDController mPidRotation = new PIDController(1, 0, 0);

    private final double ApprochMinTime;

    // private double kP = 1;
    // private double kD = 0;

    // private double lastdelta = 0;

    public RotateCommand(CommandSwerveDrivetrain mDrivetrainSubsystem, DoubleSupplier RotateRadianAbslute, double ApprochMinTimeSecond){

        drivetrain = mDrivetrainSubsystem;

        this.RotateRadianAbslute = RotateRadianAbslute;
        mPidRotation.setSetpoint(0);
        ApprochMinTime = ApprochMinTimeSecond;

        //lastdelta = drivetrain.getHeading().getRadians()-RotateRadianAbslute.getAsDouble();

    }

    @Override
    public void execute() {
        mPidRotation.setSetpoint(RotateRadianAbslute.getAsDouble());
        //double delta = Util.map(-Math.PI, Math.PI,drivetrain.getHeading().getRadians()-RotateRadianAbslute.getAsDouble());


        ChassisSpeeds targetVelocity = drivetrain.getState().speeds;
        //targetVelocity.omegaRadiansPerSecond = /*Util.map(-Math.PI,Math.PI,*/RotateRadianAbslute.getAsDouble()-drivetrain.getHeading().getRadians();//);//delta * kP + (delta-lastdelta)* kD;
        //CConsole.stdout.log("Heading: ",drivetrain.getHeading().getRadians(),"(Desired=",RotateRadianAbslute.getAsDouble(),")");
        //CConsole.stdout.log("RotV=",targetVelocity.omegaRadiansPerSecond);

        targetVelocity.omegaRadiansPerSecond = calcRotVel();
        drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(targetVelocity));;
        //lastdelta = delta;
    }


    @Override
    public void end(boolean interrupted) {
        drivetrain.setTargetVelocity(new ChassisSpeeds());
    }

    public double calcRotVel()
    {
        double rv;
        rv = Math.atan(mPidRotation.calculate(Util.map(-Math.PI, Math.PI, drivetrain.getHeading().getRadians() - RotateRadianAbslute.getAsDouble()),0))/ApprochMinTime;
        //CConsole.stdout.log("Delta Rotation:",Util.map(-Math.PI, Math.PI, drivetrain.getHeading().getRadians() - RotateRadianAbslute.getAsDouble()),"/ Vel",rv);
        if(Util.epsilonEquals(rv, 0,0.02 / ApprochMinTime))
        {
            rv = 0;
        }
        return rv;
    }
}