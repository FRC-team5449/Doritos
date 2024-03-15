package com.team5449.frc2024.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.team5449.frc2024.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
    private static final double LOWEST_DISTANCE = Units.feetToMeters(1.0);

    private final VisionIO[] cameras;
    private final VisionIOInputsAutoLogged[] inputs;

    private final List<VisionSubsystem.PoseAndTimestamp> results = new ArrayList<>();

    private int acceptableTagID;
    private boolean useSingleTag = false;

    public VisionSubsystem(VisionIO[] cameras) {
        this.cameras = cameras;
        inputs = new VisionIOInputsAutoLogged[cameras.length];

        for (int i = 0; i < cameras.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput("useSingleTag", useSingleTag);

        // clear results from last periodic
        results.clear();

        for (int i = 0; i < inputs.length; i++) {
            // update and process new inputs
            cameras[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + cameras[i].getName() + "/Inputs", inputs[i]);

            if (inputs[i].hasTarget
                    && inputs[i].isNew
                    && !DriverStation.isAutonomous()
                    && inputs[i].maxDistance < LOWEST_DISTANCE) {
                if (useSingleTag) {
                    if (inputs[i].singleIDUsed == acceptableTagID) {
                        processVision(i);
                    }
                } else {
                    processVision(i);
                }
            }
        }

        Logger.recordOutput("Vision/ResultCount", results.size());

        SmartDashboard.putNumber("Stage Distance", getStageDistance(0)+0.505);
    }

    public double getStageDistance(int camera){
        return Math.abs(inputs[camera].translationToTargetX);// - Constants.aprilTagToStageCenterMeter;
    }

    public double getOffsetToTarget(int camera){
        return inputs[camera].rotationToTargetYaw;
    }

    public void processVision(int cameraNum) {
        // create a new pose based off the new inputs
        Pose2d currentPose =
                new Pose2d(inputs[cameraNum].x, inputs[cameraNum].y, new Rotation2d(inputs[cameraNum].rotation));
        Logger.recordOutput(cameras[cameraNum].getName() + " pose", currentPose);

        // add the new pose to a list
        results.add(new PoseAndTimestamp(currentPose, inputs[cameraNum].timestamp));
    }

    
    public List<VisionSubsystem.PoseAndTimestamp> getVisionOdometry() {
        return results;
    }

    public static class PoseAndTimestamp {
        Pose2d pose;
        double timestamp;

        public PoseAndTimestamp(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }

    public void setUseSingleTag(boolean useSingleTag) {
        setUseSingleTag(useSingleTag, 0);
    }

    public void setUseSingleTag(boolean useSingleTag, int acceptableTagID) {
        this.useSingleTag = useSingleTag;
        this.acceptableTagID = acceptableTagID;
    }

    public void setReferencePose(Pose2d pose) {
        for (VisionIO io : cameras) {
            io.setReferencePose(pose);
        }
    }

    public double getMinDistance(int camera) {
        return inputs[camera].minDistance;
    }

    public boolean getTargetDetected(int camera) {
        return inputs[camera].hasTarget;
    }
}
