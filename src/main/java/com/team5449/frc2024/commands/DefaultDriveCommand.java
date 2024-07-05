package com.team5449.frc2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;
import com.team5449.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDriveCommand extends Command{
    private final DrivetrainSubsystem drivetrain;

    private final double ROTATION_COEFFICIENT = 1;

    public static final double ANGULAR_VELOCITY_COEFFICIENT = 0.085;

    private final DoubleSupplier xVelocitySupplier;
    
    private final DoubleSupplier yVelocitySupplier;
    
    private final DoubleSupplier angularVelocitySupplier;

    private final BooleanSupplier joystickButton;

    private final BooleanSupplier resetPigeon;

    private ChassisSpeeds targetVelocity;

    private boolean preValue = true;

    public DefaultDriveCommand(DrivetrainSubsystem subsystem,       
        DoubleSupplier xVelocitySupplier,
        DoubleSupplier yVelocitySupplier,
        DoubleSupplier angularVelocitySupplier,
        BooleanSupplier joystickButton,
        BooleanSupplier resetPigeon){

        drivetrain = subsystem;

        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;
        this.joystickButton = joystickButton;
        this.resetPigeon = resetPigeon;

        targetVelocity = new ChassisSpeeds();
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double xVelocity;
        double yVelocity;
        double angularVelocity;
        boolean isFieldRelative;
        boolean setGyro;

        xVelocity = xVelocitySupplier.getAsDouble();

        yVelocity = yVelocitySupplier.getAsDouble();

        angularVelocity =
                angularVelocitySupplier.getAsDouble() *  ROTATION_COEFFICIENT;

        if(!preValue && joystickButton.getAsBoolean()) preValue = true;
        else if(preValue && joystickButton.getAsBoolean()) preValue = false;

        isFieldRelative = preValue;

        setGyro = resetPigeon.getAsBoolean();

        Logger.recordOutput("Drive/Rotation offset", drivetrain.getAngularVelocity() * ANGULAR_VELOCITY_COEFFICIENT);

        
        if(isFieldRelative){
            targetVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVelocity,
                    yVelocity,
                    angularVelocity,
                    drivetrain.getHeading());
        }
        else{
            targetVelocity = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
        }

        drivetrain.setTargetVelocity(targetVelocity);

        if(setGyro){
            drivetrain.resetHeading();
        }

        SmartDashboard.putNumber("Rotation", drivetrain.getHeading().getDegrees());
        SmartDashboard.putString("FieldRelative", isFieldRelative ? "Yes" : "No");
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setTargetVelocity(new ChassisSpeeds());
    }
}
