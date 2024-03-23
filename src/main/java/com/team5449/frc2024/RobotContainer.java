// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.schema.Joystick;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team5449.frc2024.Constants.Ports;
import com.team5449.frc2024.autos.autocommands.WaitCommand;
import com.team5449.frc2024.commands.AmpCommand;
import com.team5449.frc2024.commands.ArmPoseCommand;
import com.team5449.frc2024.commands.AutoAlign;
import com.team5449.frc2024.commands.ClimbCommand;
import com.team5449.frc2024.commands.DefaultDriveCommand;
import com.team5449.frc2024.commands.IntakeCommand;
import com.team5449.frc2024.commands.OrientToTargetCommand;
import com.team5449.frc2024.commands.OuttakeCommand;
import com.team5449.frc2024.commands.RotateCommand;
import com.team5449.frc2024.commands.ShootCommand;
import com.team5449.frc2024.commands.ArmPoseCommand.ArmSystemState;
import com.team5449.frc2024.subsystems.CalcRotationWithUnitCircleData;
import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;
import com.team5449.frc2024.subsystems.drive.GyroIOPigeon;
import com.team5449.frc2024.subsystems.drive.SwerveModuleIOFalconPro;
import com.team5449.frc2024.subsystems.score.Arm;
import com.team5449.frc2024.subsystems.score.Climber;
import com.team5449.frc2024.subsystems.score.Intake;
import com.team5449.frc2024.subsystems.score.Shooter;
import com.team5449.frc2024.subsystems.vision.Led;
import com.team5449.frc2024.subsystems.vision.Led.Color;
import com.team5449.frc2024.subsystems.vision.VisionIO;
import com.team5449.frc2024.subsystems.vision.VisionIOLimelight;
import com.team5449.frc2024.subsystems.vision.VisionSubsystem;
import com.team5449.lib.util.TimeDelayedBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final VisionSubsystem vision;
  private final Shooter shooter;
  private final Intake intake;
  private final Arm arm;
  private final Climber climber;

  private final ArmPoseCommand armPoseCommand;

  private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter omegaLimiter = new SlewRateLimiter(2);

  private TimeDelayedBoolean resetGyroBoolean = new TimeDelayedBoolean();

  public final XboxController mDriverController = new XboxController(0);
  public final XboxController mOperatorController = new XboxController(1);
  //private final edu.wpi.first.wpilibj.Joystick mDriverJoystick = new edu.wpi.first.wpilibj.Joystick(0);
  //public final CommandXboxController mOperatorController = new CommandXboxController(1);

  private final DigitalInput noteStored = new DigitalInput(0);

  private final SendableChooser<Command> mAutoChooser;

  private final OrientToTargetCommand mOrientToTargetCommand;
  private final AutoAlign mAutoAlignCommand;

  private final CalcRotationWithUnitCircleData mCircleData;

  private final RotateCommand mRotateCommand;

  private final GyroIOPigeon mPigeon;

  public RobotContainer() {

    
    vision = new VisionSubsystem(new VisionIO[]{
      new VisionIOLimelight("limelight", new Transform3d())
    });

    mPigeon = new GyroIOPigeon();
    drivetrainSubsystem = new DrivetrainSubsystem(
      mPigeon,
      new SwerveModuleIOFalconPro(Ports.kFrontLeftMotorId, Ports.kFrontLeftAziId, Ports.kFrontLeftEncoderId, Ports.kCANBusFDName, Constants.kFrontLeftEncoderOffset, false),
      new SwerveModuleIOFalconPro(Ports.kFrontRightMotorId, Ports.kFrontRightAziId, Ports.kFrontRightEncoderId, Ports.kCANBusFDName, Constants.kFrontRightEncoderOffset, false),
      new SwerveModuleIOFalconPro(Ports.kBackLeftMotorId, Ports.kBackLeftAziId, Ports.kBackLeftEncoderId, Ports.kCANBusFDName, Constants.kBackLeftEncoderOffset, true),
      new SwerveModuleIOFalconPro(Ports.kBackRightMotorId, Ports.kBackRightAziId, Ports.kBackRightEncoderId, Ports.kCANBusFDName, Constants.kBackRightEncoderOffset, true),
      vision
    );

    arm = new Arm();
    climber=new Climber();
    armPoseCommand = new ArmPoseCommand(arm, vision);
    arm.setDefaultCommand(armPoseCommand);


    mCircleData = new CalcRotationWithUnitCircleData(mOperatorController::getRightX, ()->-mOperatorController.getRightY(), drivetrainSubsystem.getHeading().getRadians(), 0.5);
    mRotateCommand = new RotateCommand(drivetrainSubsystem, mCircleData::calculate,0.25);

    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      drivetrainSubsystem,
      () -> -adjustJoystickValue(xLimiter.calculate(mDriverController.getLeftY())) * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
      () -> -adjustJoystickValue(yLimiter.calculate(mDriverController.getLeftX())) * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
      () -> -adjustJoystickValue(omegaLimiter.calculate(mDriverController.getRightX())) * drivetrainSubsystem.getMaxAngularVelocityRadPerSec()/* + mRotateCommand.calcRotVel()*/,
      () -> mDriverController.getBButtonPressed(),
      () -> {boolean reset = resetGyroBoolean.update(mDriverController.getAButton(), 0.2);if(reset){mCircleData.reset();}return reset;}));

      drivetrainSubsystem.setPathAuto();


    shooter = new Shooter();
    intake = new Intake();

    mOrientToTargetCommand = new OrientToTargetCommand(drivetrainSubsystem, vision);
    mAutoAlignCommand = new AutoAlign(drivetrainSubsystem, vision);

    pathPlannerRegisterCommand();

    mAutoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Builder Auto Chooser", mAutoChooser);

    mAutoChooser.onChange((mCommand) -> System.out.println(mCommand.getName()));

    configureBindings();
  }

  private static double adjustJoystickValue(double value) {
      value = MathUtil.applyDeadband(value, 0.01);
      value = value*value*value;//Math.copySign(value * value, value);
      return value;
  }

  private void configureBindings() {
    BooleanSupplier conditionShoot = new Trigger(mOperatorController::getAButton);
    BooleanSupplier conditionIntake = new Trigger(mOperatorController::getBButton);
    BooleanSupplier conditionReload = new Trigger(mOperatorController::getXButton);
    BooleanSupplier conditionGoAMP = new Trigger(mOperatorController::getYButton);
    //BooleanSupplier conditionHasTarget = ()->mColorSensor.getTarget()==new Constants.checkTarget[]{Constants.checkTarget.HASTARGET};

    new Trigger(conditionShoot).onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.SHOOTING))).whileTrue(new ShootCommand(shooter, () -> armPoseCommand.getArmState() == ArmSystemState.SHOOTING, 80));//.whileTrue(mAutoAlignCommand);//.o

    new Trigger(conditionIntake).onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.INTAKE)));
    new Trigger(() -> armPoseCommand.getArmState() == ArmSystemState.INTAKE && conditionIntake.getAsBoolean()==true).whileTrue(new IntakeCommand(shooter, intake));

    new Trigger(conditionReload).onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.OUTTAKE))).whileTrue(new OuttakeCommand(shooter, intake));

    new Trigger(conditionGoAMP).onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.AMP))).whileTrue(new AmpCommand(shooter, () -> armPoseCommand.getArmState() == ArmSystemState.AMP,-40));

    new Trigger(() -> mOperatorController.getLeftTriggerAxis() == 1).onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.PRECLIMB)));
    new Trigger(() -> mOperatorController.getRightTriggerAxis() == 1).onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.CLIMB)));

    new Trigger(mOperatorController::getLeftBumper).whileTrue(new ClimbCommand(climber, 0.7));

    //new Trigger(mDriverController::getLeftBumper).onTrue(new InstantCommand(() -> shooter.transit(0.5))).onFalse(new InstantCommand(() -> shooter.transit(0)));

    new Trigger(mOperatorController::getRightBumper).whileTrue(new ClimbCommand(climber, -0.7));
    new Trigger(() -> mOperatorController.getPOV() == 180).onTrue(new InstantCommand(()->armPoseCommand.setPose(ArmSystemState.TRAP)));
    new Trigger(() -> mOperatorController.getPOV() == 0).whileTrue(new AmpCommand(shooter, ()->true, -60));
    new Trigger(() -> mOperatorController.getPOV() == 270).onTrue(new InstantCommand(()->armPoseCommand.setPose(ArmSystemState.PRETRAP)).alongWith(
      new InstantCommand(()->shooter.setOpenLoop(-0.2, false))))
      .onFalse(new InstantCommand(()->shooter.setOpenLoop(0, false)));

    //new Trigger(mOperatorController::getLeftStickButton).onTrue(new InstantCommand(() -> shooter.transit(1))).onFalse(new InstantCommand(() -> shooter.transit(0)));

    new Trigger(() -> mDriverController.getRightBumper()).whileTrue(mAutoAlignCommand);

    new Trigger(noteStored::get).onTrue(new WaitCommand(
      new InstantCommand(() -> {
        mDriverController.setRumble(RumbleType.kBothRumble, 0.5);
        mOperatorController.setRumble(RumbleType.kBothRumble, 0.5);
      }), 0.5
    ).andThen(
      new InstantCommand(() -> {
        mDriverController.setRumble(RumbleType.kBothRumble, 0);
        mOperatorController.setRumble(RumbleType.kBothRumble, 0);
      })
    ));

    new Trigger(mOperatorController::getLeftStickButton).onTrue(new InstantCommand(() -> armPoseCommand.offsetBy(0.005)));
    new Trigger(mOperatorController::getRightStickButton).onTrue(new InstantCommand(() -> armPoseCommand.offsetBy(-0.005)));
    new Trigger(mOperatorController::getLeftStickButton).and(mOperatorController::getRightStickButton).onTrue(new InstantCommand(armPoseCommand::resetOffset));

    
  }


  

  public Command getAutonomousCommand() {
    final Command nowSelected = mAutoChooser.getSelected();
    String name = nowSelected.getName();
    if(!nowSelected.getName().equals("InstantCommand"))
    {
      JSONParser parser = new JSONParser();
      try{
        Object obj = parser.parse(new FileReader("/home/lvuser/deploy/pathplanner/autos/"+name+".auto"));
        JSONObject autoJsonObj = (JSONObject)obj;
        Pose2d startPose = AutoBuilder.getStartingPoseFromJson((JSONObject)autoJsonObj.get("startingPose"));
        System.out.println("Inital heading: "+startPose.getRotation());
        drivetrainSubsystem.resetHeading(DriverStation.getAlliance().get()==Alliance.Blue ? -startPose.getRotation().getRadians() : -180-startPose.getRotation().getRadians());
        //TODO: test if works
      }
      catch (FileNotFoundException e) {
        e.printStackTrace();
      }
      catch (IOException e) {
        e.printStackTrace();
      }
      catch (ParseException e) {
        e.printStackTrace();
      }
    }
    return nowSelected;
  }

  public DrivetrainSubsystem getDrivetrainSubsystem(){
    return drivetrainSubsystem;
  }

  private final TimeDelayedBoolean mNoteOutHelper = new TimeDelayedBoolean();
  private final BooleanSupplier isNoteOut = () -> {
    final boolean ans = mNoteOutHelper.update(!noteStored.get(), 0.2);
    if(!noteStored.get())
    {
      System.out.println("Shoot in SENSOR in "+Timer.getFPGATimestamp());
    }
    return ans;
  };
  private final BooleanConsumer mPrintNote = (e) -> {
    if(e){
      System.out.println("Shoot in MOTOR in "+Timer.getFPGATimestamp());
    }
  };
  private void pathPlannerRegisterCommand(){
    NamedCommands.registerCommand("Intake", new WaitCommand(new IntakeCommand(shooter, intake), 10, noteStored::get).alongWith(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.INTAKE))));
    NamedCommands.registerCommand("NearShoot", new WaitCommand(new ShootCommand(shooter, () -> armPoseCommand.getArmState() == ArmSystemState.AUTOSHOOT, 65, mPrintNote), 10, isNoteOut).alongWith(new InstantCommand(() -> armPoseCommand.setAutoShootPosition(0.25))));
    NamedCommands.registerCommand("MiddleShoot", new WaitCommand(new ShootCommand(shooter, () -> armPoseCommand.getArmState() == ArmSystemState.AUTOSHOOT, 75, mPrintNote), 10, isNoteOut).alongWith(new InstantCommand(() -> armPoseCommand.setAutoShootPosition(0.22))));
    NamedCommands.registerCommand("FarShoot", new WaitCommand(new ShootCommand(shooter, () -> armPoseCommand.getArmState() == ArmSystemState.AUTOSHOOT, 80, mPrintNote), 10, isNoteOut).alongWith(new InstantCommand(() -> armPoseCommand.setAutoShootPosition(0.196))));
    NamedCommands.registerCommand("Arm Down", new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.ARMDOWN)));
  }

}
