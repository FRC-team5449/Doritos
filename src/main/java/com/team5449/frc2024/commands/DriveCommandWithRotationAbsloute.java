// package com.team5449.frc2024.commands;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import org.littletonrobotics.junction.Logger;

// import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;
// import com.team5449.lib.util.Util;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.util.function.BooleanConsumer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// public class DriveCommandWithRotationAbsloute extends Command{
//     private final DrivetrainSubsystem drivetrain;

//     public static final double ANGULAR_VELOCITY_COEFFICIENT = 0.085;

//     private final DoubleSupplier xVelocitySupplier;
    
//     private final DoubleSupplier yVelocitySupplier;
    
//     private final RotateCommand mRotateCommand;

//     private final BooleanSupplier joystickButton;

//     private final BooleanSupplier resetPigeon;

//     private final BooleanConsumer onReset;

//     private ChassisSpeeds targetVelocity;

//     private boolean preValue = false;

//     PIDController mPidRotation = new PIDController(1, 0, 0);

//     public DriveCommandWithRotationAbsloute(DrivetrainSubsystem subsystem,       
//         DoubleSupplier xVelocitySupplier,
//         DoubleSupplier yVelocitySupplier,
//         DoubleSupplier rotationRadianSupplier,
//         BooleanConsumer onReset,
//         BooleanSupplier joystickButton,
//         BooleanSupplier resetPigeon){

//         drivetrain = subsystem;

//         this.xVelocitySupplier = xVelocitySupplier;
//         this.yVelocitySupplier = yVelocitySupplier;
//         this.mRotateCommand = new RotateCommand(subsystem, rotationRadianSupplier,0.25);
//         this.joystickButton = joystickButton;
//         this.resetPigeon = resetPigeon;
//         this.onReset = onReset;

//         targetVelocity = new ChassisSpeeds();
//         addRequirements(drivetrain);
//     }

//     @Override
//     public void execute() {
//         double xVelocity;
//         double yVelocity;
//         double angularVelocity;
//         boolean isFieldRelative;
//         boolean setGyro;

//         xVelocity = xVelocitySupplier.getAsDouble();

//         yVelocity = yVelocitySupplier.getAsDouble();

//         angularVelocity = mRotateCommand.calcRotVel();
//         if(Util.epsilonEquals(angularVelocity, 0, 0.1))
//         {
//             angularVelocity = 0;
//         }

//         if(!preValue && joystickButton.getAsBoolean()) preValue = true;
//         else if(preValue && joystickButton.getAsBoolean()) preValue = false;

//         isFieldRelative = preValue;

//         setGyro = resetPigeon.getAsBoolean();

//         Logger.recordOutput("Drive/Rotation offset", drivetrain.getAngularVelocity() * ANGULAR_VELOCITY_COEFFICIENT);

        
//         if(isFieldRelative){
//             targetVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
//                     xVelocity,
//                     yVelocity,
//                     angularVelocity,
//                     drivetrain.getHeading());
//         }
//         else{
//             targetVelocity = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
//         }

//         drivetrain.setTargetVelocity(targetVelocity);

//         if(setGyro){
//             drivetrain.resetHeading();
//             onReset.accept(setGyro);
//         }

//         SmartDashboard.putNumber("Rotation", drivetrain.getHeading().getDegrees());
//         SmartDashboard.putString("FieldRelative", isFieldRelative ? "Yes" : "No");
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrain.setTargetVelocity(new ChassisSpeeds());
//     }
// }
