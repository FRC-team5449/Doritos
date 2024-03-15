package com.team5449.frc2024.commands;

import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;
import com.team5449.frc2024.subsystems.score.Intake;
import com.team5449.frc2024.subsystems.score.Shooter;
import com.team5449.frc2024.subsystems.vision.VisionSubsystem;
import com.team5449.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class OrientToTargetCommand extends Command{
private final DrivetrainSubsystem mDrive;
  private final VisionSubsystem mVision;

  private final PIDController omegaController;
  /** Creates a new IntakeCommand. */
  public OrientToTargetCommand(DrivetrainSubsystem drive, VisionSubsystem vision) {
    mDrive = drive;
    mVision = vision;
    omegaController = new PIDController(2, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    omegaController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double omegaVelocity = omegaController.calculate(mVision.getOffsetToTarget(0));

    double omegaVelocity = -mVision.getOffsetToTarget(0);
    SmartDashboard.putNumber("Omega Velocity", Units.degreesToRadians(omegaVelocity));
    if(!mVision.getTargetDetected(0)){
        System.err.println("No target AprilTag detected.");
    }
    mDrive.setTargetVelocity(new ChassisSpeeds(0, 0, Units.degreesToRadians(omegaVelocity)));
    
    //System.out.println("111");
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   //mDrive.setTargetVelocity(new ChassisSpeeds());

   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return omegaController.atSetpoint();
    return Util.epsilonEquals(0, mVision.getOffsetToTarget(0), 0.4);
  }
}
