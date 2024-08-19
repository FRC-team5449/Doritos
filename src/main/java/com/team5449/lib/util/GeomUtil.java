package com.team5449.lib.util;

import com.team5449.frc2024.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GeomUtil {
    public static Pose2d inverse(Pose2d pose){
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(
            pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse
        );
    }
    public static Translation3d GetStageTranslation(){
        return DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue?Constants.ShootTargetPosBlue:Constants.ShootTargetPosRed;
    }
}
