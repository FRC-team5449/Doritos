package com.team5449.frc2024.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public double x;
        public double y;
        public double rotation;
        public double timestamp;
        public boolean isNew; // is new pose

        public double maxAmbiguity;
        public double maxDistance;
        public double minDistance;

        public boolean hasTarget = false;
        public int singleIDUsed;
        public double singleIDUsedDouble;

        public double translationToTargetX;
        public double translationToTargetY;

        public double rotationToTargetYaw;
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default String getName() {
        return "";
    }

    default void setReferencePose(Pose2d pose) {}
    
}
