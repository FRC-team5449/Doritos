package com.team5449.frc2024;

import java.util.function.BooleanSupplier;

import com.team5449.lib.util.ControllerUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Constants {

    public static final boolean isOfficialbot = true;

    public static final double kLooperDt = 0.02;
    public static final double kEpsilon = 1e-12;

    //Gyro Pigeon
    public static final double pigeonMountPoseYaw = 0;
    public static final double pigeonMountPoseRoll = 0;
    public static final double pigeonMountPosePitch = 0;
    public static final double pigeonError = 0;

    //Drivetrain 
    public static final double wheelBaseMeters = 0.49;
    public static final double trackBaseMeters = 0.54;
    public static final double driveBaseRadius = Math.hypot(wheelBaseMeters / 2, trackBaseMeters / 2);

    public static final double maxVelocityMeterPerSec = 8;
    public static final double maxAccelerationMeterPerSecSq = 100;


    public static final double maxAutoVelocityMeterPerSec = 2;
    public static final double maxAutoAccelerationMeterPerSecSq = 100;

    public static final double maxAngularVelocityRadPerSec = 10; //maxVelocityMeterPerSec / Math.hypot(wheelBaseMeters / 2, trackBaseMeters / 2);

    public static final int driveCurrentLimit = 70;
    public static final int steerCurrentLimit = 30;

    public static final double drivekP = 0.2;
    public static final double drivekI = 0;
    public static final double drivekD = 0;
    public static final double drivekV = 0.12;

    public static final Translation2d frontLeftLocation = new Translation2d(Constants.wheelBaseMeters / 2, Constants.trackBaseMeters / 2);
    public static final Translation2d frontRightLocation = new Translation2d(Constants.wheelBaseMeters / 2, -Constants.trackBaseMeters / 2);
    public static final Translation2d backLeftLocation = new Translation2d(-Constants.wheelBaseMeters / 2, Constants.trackBaseMeters / 2);
    public static final Translation2d backRightLocation = new Translation2d(-Constants.wheelBaseMeters / 2, -Constants.trackBaseMeters / 2);
    public static final SwerveDriveKinematics swerveDriveKinematics =
                new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    public static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));

    //254 PID
    public static final double kMk4DriveVelocityKp = 0.02 * 12;
    public static final double kMk4DriveVelocityKi = 0.0;
    public static final double kMk4DriveVelocityKd = 0.000002 * 12;
    public static final double kMk4DriveVelocityKv = 1 / 101.98 * 12;
    public static final double kMk4DriveVelocityKs = 0.8;


    public static final double turnkP = 3.0005;
    public static final double turnkI = 0;
    public static final double turnkD = 0.0004;
    public static final double turnkA = 0;
    public static final double turnkS = 0.05;
    public static final double turnkV = 0.1;

    public static final double turnkP1 = 100;
    public static final double turnkI1 = 0;
    public static final double turnkd1 = 0;
    public static final double turnkA1 = 0;
    public static final double turnkS1 = 0;
    public static final double turnkV1 = 0;
    public static final double turnkD1 = 0;

    public static final double wheelRadiMeters = 0.05;
    public static final double driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

    public static final double drivePositionCoefficient = (2 * Math.PI * wheelRadiMeters) / driveGearRatio;

    public static final double driveVelocityCoefficient = drivePositionCoefficient;

    public static final double steerGearRatio = 7.0 / 150.0;

    public static final double steerPositionCoefficient = 2.0 * Math.PI * steerGearRatio;

    //CANCoder Positive Direction Offset
    public static final double kFrontLeftEncoderOffset = 0.9765625;
    public static final double kFrontRightEncoderOffset = 0.576171875;
    public static final double kBackLeftEncoderOffset = 0.34130859375;
    public static final double kBackRightEncoderOffset = 0.924560546875;

    //Vision Constants
    public static final double aprilTagToStageCenterMeter = 0.505;
    public static final double armToSpeakerVerticalMeter = 2-0.6;

    //Arm Constants
    public static final double shooterAllowanceSpeed = 0;
    public static final double maxArmPosition = 0.5;
    public static final double minArmPosition = 0.02;
    public static final double armGearRatio = 1 / 15;
    public static final double armPositionCoefficient = 2.0 * Math.PI * armGearRatio;

    public static final double autoNoteArmPose1 = 0.0;
    public static final double autoNoteArmPose2 = 0.0;
    public static final double autoNoteArmPose3 = 0.0;

    //Shooter Constants
    public static final double kShooterReadySpeed = 0;



    public static class Ports {
        public static final String kCANBusFDName = isOfficialbot ? "canivore" : "rio";
        public static final String kCANBusDefault = "rio";
        public static final String visionName = "limelight";
        
        public static final int kPigeonId = 5;

        public static final int kFrontLeftMotorId = 1;
        public static final int kFrontLeftAziId = 21;
        public static final int kFrontLeftEncoderId = 31;

        public static final int kFrontRightMotorId = 4;
        public static final int kFrontRightAziId = 24;
        public static final int kFrontRightEncoderId = 34;

        public static final int kBackLeftMotorId = 3;
        public static final int kBackLeftAziId = 23;
        public static final int kBackLeftEncoderId = 33;

        public static final int kBackRightMotorId = 2;
        public static final int kBackRightAziId = 22;
        public static final int kBackRightEncoderId = 32;

        public static final int kShooterLeftId = 41;
        public static final int kShooterRightId = 40;

        public static final int kShooterUpId = 40;
        public static final int kShooterLowId = 41;

        public static final int kIntake1Id = 6;

        public static final int kTransId = 8;

        public static final int kArmMasterId = 13;
        public static final int kArmSlaveId = 14;

        public static final int kClimbLftId = 7;
        

    }

    public static final double ControlTimeout = 0.05;

    public static class ControlConds{
        public static final int DriverPort = 0;
        public static final int OperatorPort = 1;
        // Driver
        static{
            ControllerUtil.setControlPort(DriverPort);
        }
        // public static final long SwitchHeadNReset = ControllerUtil.GetXboxVal("B"); // It is calling getBButton instead of getBButtonPressed
        public static final long AutoAlignStage = ControllerUtil.GetXboxVal("Y");
        public static final long CounterClkwRotatePos90Deg = ControllerUtil.GetXboxVal("LeftBumper");
        public static final long ClkwRotatePos90Deg = ControllerUtil.GetXboxVal("RightBumper");

        // Operator
        static{
            ControllerUtil.setControlPort(OperatorPort);
        }
        public static final long shoot = ControllerUtil.GetXboxVal("A", 0);
        public static final long intake = ControllerUtil.GetXboxVal("B");
        public static final long reload = ControllerUtil.GetXboxVal("X");
        public static final long amp = ControllerUtil.GetXboxVal("Y");
        public static final long scalestring1 = ControllerUtil.GetXboxVal("LeftBumper");
        public static final long scalestring2 = ControllerUtil.GetXboxVal("RightBumper");
        public static final long overshoot = reload | scalestring1 | scalestring2;
        public static final long offsetArmUp = ControllerUtil.GetXboxVal("LeftStick");
        public static final long offsetArmDown = ControllerUtil.GetXboxVal("RightStick");
        public static final long ResetArmOffset = offsetArmUp | offsetArmDown;
        public static final long forceShoot = shoot | reload;
        public static final long forceIntake = intake | scalestring1 | scalestring2;

    }

    public static enum checkTarget
    {
        HASTARGET, EMPTY
    }
}
