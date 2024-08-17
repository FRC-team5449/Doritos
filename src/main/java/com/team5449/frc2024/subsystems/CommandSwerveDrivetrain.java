package com.team5449.frc2024.subsystems;

import java.util.Arrays;
import java.util.function.Supplier;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team5449.frc2024.subsystems.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.team5449.frc2024.DriveConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Field2d field=new Field2d();
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private static final String LLM = "limelight-mid";
    private static final String LLR = "limelight-right";

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private static final double[] stdDevFactors = new double[] {10, 0.6, 1.0, 1.2};
    private static final double xySdCoef = 0.5;
    private static final double thetaSdCoef = 10000000;

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // PoseEstimate estimate=LimelightHelpers.getBotPoseEstimate_wpiBlue(LLname);
        // addVisionMeasurement(new Pose2d(0,0,estimate.pose.getRotation()), OdometryUpdateFrequency, VecBuilder.fill(1000000, 1000000, 0));
        AutoBuilder.configureHolonomic(
            () -> getState().Pose,
            this::seedFieldRelative,
            () -> m_kinematics.toChassisSpeeds(getState().ModuleStates),
            speeds->setControl(new ApplyChassisSpeeds().withSpeeds(speeds)),
            new HolonomicPathFollowerConfig(
                new PIDConstants(5,0,0),
                new PIDConstants(5,0,0),
                DriveConstants.kSpeedAt12VoltsMps,
                DriveConstants.kDriveBaseRadiusInches*0.0254,
                new ReplanningConfig(true, true)),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red,
            this);
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        this(driveTrainConstants, 0, VecBuilder.fill(.1, 0.1, .1), VecBuilder.fill(.9, .9, .9), modules);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }
    public void resetFieldCentricForward(){
        
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    private final void reportVision(){
        Translation2d currentTranslation=getState().Pose.getTranslation();
        PoseEstimate estimateMid1=LimelightHelpers.getBotPoseEstimate_wpiBlue(LLM);
        PoseEstimate estimateMid2=LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LLM);
        PoseEstimate estimateRight1=LimelightHelpers.getBotPoseEstimate_wpiBlue(LLR);
        PoseEstimate estimateRight2=LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LLR);
        if(estimateMid1!=null&&estimateMid2!=null&&estimateMid1.tagCount!=0&&estimateMid1.avgTagDist<4){
            if(estimateRight1!=null&&estimateRight2!=null&&estimateRight1.tagCount!=0&&estimateRight1.avgTagDist<4){
                double sd=(Math.pow(estimateMid1.avgTagDist/estimateMid1.tagCount, 2)+Math.pow(estimateRight1.avgTagDist/estimateRight1.tagCount, 2))*stdDevFactors[0]/2;
                addVisionMeasurement(
                    new Pose2d(currentTranslation.nearest(Arrays.asList(estimateMid1.pose.getTranslation().nearest(Arrays.asList(estimateRight1.pose.getTranslation(),estimateRight2.pose.getTranslation())),estimateMid2.pose.getTranslation().nearest(Arrays.asList(estimateRight1.pose.getTranslation(),estimateRight2.pose.getTranslation())))),new Rotation2d()),
                estimateMid1.timestampSeconds,VecBuilder.fill(sd*xySdCoef, sd*xySdCoef, sd*thetaSdCoef));
            }else{
                double sd=Math.pow(estimateMid1.avgTagDist/estimateMid1.tagCount, 2)*stdDevFactors[0];
                addVisionMeasurement(new Pose2d(currentTranslation.nearest(Arrays.asList(estimateMid1.pose.getTranslation(),estimateMid2.pose.getTranslation())),new Rotation2d()), estimateMid1.timestampSeconds, VecBuilder.fill(sd*xySdCoef, sd*xySdCoef, sd*thetaSdCoef));
            }
        }else if(estimateRight1!=null&&estimateRight2!=null&&estimateRight1.tagCount!=0&&estimateRight1.avgTagDist<4){
            double sd=Math.pow(estimateRight1.avgTagDist/estimateRight1.tagCount, 2)*stdDevFactors[0];
            addVisionMeasurement(new Pose2d(currentTranslation.nearest(Arrays.asList(estimateRight1.pose.getTranslation(),estimateRight2.pose.getTranslation())),new Rotation2d()), estimateRight1.timestampSeconds, VecBuilder.fill(sd*xySdCoef, sd*xySdCoef, sd*thetaSdCoef));
        }
    }
    @Override
    public void periodic() {
        reportVision();
        field.setRobotPose(getState().Pose);
        SmartDashboard.putData(field);
        
                
                /* Periodically try to apply the operator perspective */
                /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
                /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
}
