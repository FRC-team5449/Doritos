package com.team5449.frc2024.subsystems.drive;

import java.util.ArrayList;
import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team5449.frc2024.Constants;
import com.team5449.frc2024.subsystems.vision.VisionSubsystem;
import com.team5449.frc2024.subsystems.vision.VisionSubsystem.PoseAndTimestamp;
import com.team5449.lib.util.Util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase{
    private final VisionSubsystem visionSubsystem;

    private final double maxVelocityMetersPerSec;

    private final double maxAngularVelocityRadPerSec;

    public boolean isSlowMode = false;

    private final int odometryDetectorIndex = 0;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    // private double yawOffset;

    private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    private SwerveModuleState[] optimizedSwerveModuleStates = new SwerveModuleState[4];
    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModuleState[] mMeasuredStates;

    private ChassisSpeeds targetVelocity = new ChassisSpeeds();
    private ChassisSpeeds measuredVelocity = new ChassisSpeeds();

    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator estimator;

    private StructPublisher<Pose2d> publisher1 = NetworkTableInstance.getDefault().getStructTopic("OdometryPose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault().getStructTopic("EstimatorPose", Pose2d.struct).publish();

    private final SwerveModulePosition[] swerveModulePositions;
    private final OdometryUpdateThread odometryUpdateThread;

    private final PIDConstants transPidConstants = new PIDConstants(8, 0, 0);
    private final PIDConstants rotPidConstants = new PIDConstants(1.5, 0, 0);

    private final PPHolonomicDriveController swerveFollower =
            new PPHolonomicDriveController(transPidConstants, rotPidConstants, Constants.maxVelocityMeterPerSec, Constants.driveBaseRadius);


    private class OdometryUpdateThread extends Thread {
        private BaseStatusSignal[] allSignals;
        public int successfulDataAcquisitions = 0;
        public int failedDataAcquisitions = 0;

        private LinearFilter lowpass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;
        private double averageLoopTime = 0;

        public OdometryUpdateThread() {
            ArrayList<BaseStatusSignal> signalsList = new ArrayList<>();
            allSignals = new BaseStatusSignal[(4 * 4) + 2];
            for (int i = 0; i < 4; i++) {
                signalsList.addAll(Arrays.asList(swerveModules[i].getSignals()));
            }

            signalsList.addAll(Arrays.asList(gyroIO.getSignals()));
            allSignals = signalsList.toArray(new BaseStatusSignal[0]);
        }

        @Override
        public void run() {
            for (var sig : allSignals) {
                if (sig instanceof StatusSignal) {
                    ((StatusSignal<?>) sig).setUpdateFrequency(250);
                }
            }
            while (true) {
                StatusCode status = BaseStatusSignal.waitForAll(0.1, allSignals);
                lastTime = currentTime;
                currentTime = Utils.getCurrentTimeSeconds();
                averageLoopTime = lowpass.calculate(currentTime - lastTime);
                if (status.isOK()) {
                    successfulDataAcquisitions++;
                } else {
                    failedDataAcquisitions++;
                }

                synchronized (swerveModules) {
                    synchronized (swerveModulePositions) {
                        /* Now update odometry */
                        for (int i = 0; i < 4; ++i) {
                            swerveModules[i].updateInputs();
                            swerveModulePositions[i] = swerveModules[i].getPosition();
                        }
                    }
                }

                synchronized (swerveModules){
                    synchronized (mMeasuredStates){
                        for (int i = 0; i < 4; ++i) {
                            swerveModules[i].updateInputs();
                            mMeasuredStates[i] = swerveModules[i].getMeasuredState();
                        }
                    }
                }


                synchronized (gyroIO) {
                    synchronized (gyroInputs) {
                        gyroIO.updateInputs(gyroInputs);
                    }
                }
                synchronized (odometry) {
                    synchronized (swerveModulePositions) {
                        synchronized (gyroInputs) {
                            odometry.update(Rotation2d.fromRadians(gyroInputs.yaw), swerveModulePositions);
                        }
                    }
                }

                synchronized (estimator) {
                    synchronized (swerveModulePositions) {
                        synchronized (gyroInputs) {
                            estimator.update(Rotation2d.fromRadians(gyroInputs.yaw), swerveModulePositions);
                        }
                    }
                }
            

        }
    }

        public double getTime() {
            return averageLoopTime;
        }

        public int getSuccessfulDataAcquisitions() {
            return successfulDataAcquisitions;
        }

        public int getFailedDataAcquisitions() {
            return failedDataAcquisitions;
        }
    }

    /**
     * Brains of the drivetrain subsystem. Initializes the swerve drive IOs, swerve drive modules, and swerve drive kinematics and odometry.
     */
    public DrivetrainSubsystem(
            GyroIO gyroIO,
            SwerveModuleIO frontLeftSwerveModuleIO,
            SwerveModuleIO frontRightSwerveModuleIO,
            SwerveModuleIO backLeftSwerveModuleIO,
            SwerveModuleIO backRightSwerveModuleIO,
            VisionSubsystem vision) {
        
        maxVelocityMetersPerSec = Constants.maxVelocityMeterPerSec;
        maxAngularVelocityRadPerSec = Constants.maxAngularVelocityRadPerSec;

        this.gyroIO = gyroIO;

        swerveModules = new SwerveModule[] {
            new SwerveModule(frontLeftSwerveModuleIO, "FrontLeft"),
            new SwerveModule(frontRightSwerveModuleIO, "FrontRight"),
            new SwerveModule(backLeftSwerveModuleIO, "BackLeft"),
            new SwerveModule(backRightSwerveModuleIO, "BackRight")
        };
        swerveModulePositions = new SwerveModulePosition[] {
            swerveModules[0].getPosition(),
            swerveModules[1].getPosition(),
            swerveModules[2].getPosition(),
            swerveModules[3].getPosition()
        };
        var frontLeftLocation = new Translation2d(Constants.wheelBaseMeters / 2, Constants.trackBaseMeters / 2);
        var frontRightLocation = new Translation2d(Constants.wheelBaseMeters / 2, -Constants.trackBaseMeters / 2);
        var backLeftLocation = new Translation2d(-Constants.wheelBaseMeters / 2, Constants.trackBaseMeters / 2);
        var backRightLocation = new Translation2d(-Constants.wheelBaseMeters / 2, -Constants.trackBaseMeters / 2);
        kinematics =
                new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        odometry = new SwerveDriveOdometry(
                kinematics,
                new Rotation2d(),
                new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
                },
                new Pose2d());
        estimator = new SwerveDrivePoseEstimator(
                kinematics,
                new Rotation2d(),
                new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
                },
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(40)));
        mMeasuredStates = new SwerveModuleState[]{
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

        optimizedSwerveModuleStates = new SwerveModuleState[]{
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

        visionSubsystem = vision;

        odometryUpdateThread = new OdometryUpdateThread();
        odometryUpdateThread.start();

        // yawOffset = gyroInputs.yaw;

        
    }

    @Override
    public void periodic() {

        swerveModuleStates = kinematics.toSwerveModuleStates(targetVelocity);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates,
                maxVelocityMetersPerSec); 


        measuredVelocity = kinematics.toChassisSpeeds(mMeasuredStates);

        synchronized (swerveModules) {
            for (int i = 0; i < optimizedSwerveModuleStates.length; i++) {
                optimizedSwerveModuleStates[i] =
                    SwerveModuleState.optimize(swerveModuleStates[i], swerveModules[i].getPosition().angle);
                swerveModules[i].setTargetState(optimizedSwerveModuleStates[i]);
            }
        }

        synchronized (estimator){
            if(visionSubsystem.getTargetDetected(odometryDetectorIndex) && !DriverStation.isAutonomous()){
                PoseAndTimestamp currentPoseAndTimestamp = visionSubsystem.getVisionOdometry().get(odometryDetectorIndex);
                estimator.addVisionMeasurement(currentPoseAndTimestamp.getPose(), currentPoseAndTimestamp.getTimestamp());
            }
        }

        Logger.processInputs("Drive/Gyro", gyroInputs);
        Logger.recordOutput("Drive/Angle", getPose().getRotation().getRadians());
        Logger.recordOutput("Drive/ModuleStates", swerveModuleStates);
        Logger.recordOutput("Drive/TargetChassisVelocity", new double[] {
            targetVelocity.vxMetersPerSecond, targetVelocity.vyMetersPerSecond, targetVelocity.omegaRadiansPerSecond
        });
        Logger.recordOutput("Drive/ModuleStates", swerveModuleStates); // Logging each module state
        Logger
                .recordOutput(
                        "Drive/OptimizedModuleStates",
                        optimizedSwerveModuleStates);

        Logger.recordOutput("Drive/Pose", getPose()); 


        synchronized (estimator) {
            //Logger.recordOutput("Drive/EstimatedPose", estimator.getEstimatedPosition());
            publisher2.set(estimator.getEstimatedPosition());
            SmartDashboard.putString("Estimated Pose", estimator.getEstimatedPosition().toString());
            Logger.recordOutput("EstimatorPose", estimator.getEstimatedPosition());
        }
        
        synchronized (odometry) {
            //Logger.recordOutput("Drive/OdometryPose", odometry.getPoseMeters());
            publisher1.set(odometry.getPoseMeters());
            Logger.recordOutput("OdometryPose", odometry.getPoseMeters());
        }

        Logger.recordOutput("Drive/OdometryThreadLoop", odometryUpdateThread.getTime());

        // SmartDashboard.putNumber("Front Left Speed", mMeasuredStates[0].angle.getDegrees());
        // SmartDashboard.putNumber("Front Right Speed", mMeasuredStates[1].angle.getDegrees());
        // SmartDashboard.putNumber("Back left Speed", mMeasuredStates[2].angle.getDegrees());
        // SmartDashboard.putNumber("Back Right Speed", mMeasuredStates[3].angle.getDegrees());

        SmartDashboard.putNumber("Front Left angle", swerveModules[0].getMeasuredState().angle.getDegrees());
        SmartDashboard.putNumber("Front Right angle", swerveModules[1].getMeasuredState().angle.getDegrees());
        SmartDashboard.putNumber("Back Left angle", swerveModules[2].getMeasuredState().angle.getDegrees());
        SmartDashboard.putNumber("Back Right angle", swerveModules[3].getMeasuredState().angle.getDegrees());

        SmartDashboard.putNumber("X Translation Speed", measuredVelocity.vxMetersPerSecond);
        SmartDashboard.putNumber("Y Translation Speed", measuredVelocity.vyMetersPerSecond);
        SmartDashboard.putNumber("Total Translation Speed", Math.hypot(measuredVelocity.vxMetersPerSecond, measuredVelocity.vyMetersPerSecond));
        SmartDashboard.putNumber("Rotation Speed", measuredVelocity.omegaRadiansPerSecond);

        SmartDashboard.putNumber("Set X Speed", targetVelocity.vxMetersPerSecond);
        
    }


    public ChassisSpeeds getTargetVelocity() {
        return new ChassisSpeeds(targetVelocity.vxMetersPerSecond,targetVelocity.vyMetersPerSecond,targetVelocity.omegaRadiansPerSecond);
    }

    public void setTargetVelocity(ChassisSpeeds targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setHeadingControlSpeed(double omegaRadiansPerSecond){
        targetVelocity.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    public SwerveDrivePoseEstimator getOdometry() {
        synchronized (estimator) {
            return estimator;
        }
    }

    public Pose2d getEstimatedPose(){
        synchronized(estimator){
            return estimator.getEstimatedPosition();
        }
    }

    public Pose2d getPose() {
        return getPose(true);
    }

    public Pose2d getPose(boolean visionAngle) {
        synchronized (estimator) {
            synchronized (odometry) {
                return new Pose2d(
                        estimator.getEstimatedPosition().getX(),
                        estimator.getEstimatedPosition().getY(),
                        visionAngle
                                ? estimator.getEstimatedPosition().getRotation()
                                : odometry.getPoseMeters().getRotation());
            }
        }
    }

    public PPHolonomicDriveController getSwerveFollower() {
        return swerveFollower;
    }

    public SwerveModule[] getSwerveModules() {
        synchronized (swerveModules) {
            return swerveModules;
        }
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public ChassisSpeeds getMeasuredVelocity(){
        return measuredVelocity;
    }

    public void resetPose() {
        Pose2d pose = getPose();
        resetPose(new Pose2d(pose.getX(), pose.getY(), new Rotation2d()));
    }

    public void resetPose(Pose2d poseMeters) {
        synchronized (odometry) {
            synchronized (swerveModulePositions) {
                synchronized (gyroInputs) {
                    odometry.resetPosition(new Rotation2d(gyroInputs.yaw), swerveModulePositions, poseMeters);
                }
            }
        }
        synchronized (estimator) {
            synchronized (swerveModulePositions) {
                synchronized (gyroInputs) {
                    estimator.resetPosition(new Rotation2d(gyroInputs.yaw), swerveModulePositions, poseMeters);
                }
            }
        }
    }

    public GyroIOInputsAutoLogged getGyroInputs() {
        synchronized (gyroInputs) {
            return gyroInputs;
        }
    }

    public Rotation2d getHeading(){
        // synchronized (gyroInputs){
        //     return Rotation2d.fromDegrees((Units.radiansToDegrees(gyroInputs.yaw - yawOffset) % 360)<0?(Units.radiansToDegrees(gyroInputs.yaw - yawOffset) % 360+360):(Units.radiansToDegrees(gyroInputs.yaw - yawOffset) % 360));
        // }
        return getPose().getRotation();
    }

    public void offsetHeading(double rad){ // CW: postive
        // yawOffset += rad;
        Pose2d pose = getPose();
        resetPose(new Pose2d(pose.getX(), pose.getY(), pose.getRotation().plus(Rotation2d.fromRadians(rad))));
    }

    public void resetHeading(){
        // synchronized (gyroInputs){
        //     yawOffset = gyroInputs.yaw;
        // }
        resetPose();
    }

    /**
     * Set specific angle of field (points directly to opposite alliance as 0, and CW postive direction) to zero, doesn't change the postive direction (counterclockwise)
     * @param radians The angle in radians.
     */
    public void resetHeading(double radians){
        System.out.println("Setted heading: "+radians+" rad");
        Pose2d pose = getPose();
        resetPose(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromRadians(radians)));
        // yawOffset = radians;
    }

    public Pose2d getProjectedPose(double latency) {
        ChassisSpeeds currentVelocity = kinematics.toChassisSpeeds(swerveModuleStates);

        double xSinceLastPose = currentVelocity.vxMetersPerSecond * latency;
        double ySinceLastPose = currentVelocity.vyMetersPerSecond * latency;
        double angleSinceLastPose = currentVelocity.omegaRadiansPerSecond * latency;

        Twist2d poseChanges = new Twist2d(xSinceLastPose, ySinceLastPose, angleSinceLastPose);
        return getPose().exp(poseChanges);
    }

    public double getMaxVelocityMetersPerSec() {
        return isSlowMode?maxVelocityMetersPerSec*0.1:maxVelocityMetersPerSec;
    }

    public double getMaxAngularVelocityRadPerSec() {
        return isSlowMode?maxAngularVelocityRadPerSec*0.1:maxAngularVelocityRadPerSec;
    }


    public double getAngularVelocity() {
        synchronized (gyroInputs) {
            return gyroInputs.angularVelocity;
        }
    }

    public void setPathAuto(){
        // resetHeading(-120);
        resetPose();

        AutoBuilder.configureHolonomic(
         this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getMeasuredVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setTargetVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    Constants.maxVelocityMeterPerSec, // Max module speed, in m/s
                    Constants.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    
        );
    }

    


}
