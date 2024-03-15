package com.team5449.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GeomUtil {
    public static Pose2d inverse(Pose2d pose){
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(
            pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse
        );
    }
}
