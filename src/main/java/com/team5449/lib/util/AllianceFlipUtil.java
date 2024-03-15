package com.team5449.lib.util;

import com.team5449.frc2024.FieldConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceFlipUtil {
    /**
     * Applies flip to the xCordinate if needed
     * @param xCordinate
     * @return flipped xCordinate
     */
    public static double apply(double xCordinate){
        if(shouldFlip()){
            return FieldConstants.fieldLength - xCordinate;
        }
        else{
            return xCordinate;
        }
    }

    public static Translation3d apply(Translation3d translation3d){
        if(shouldFlip()){
            return new Translation3d( apply(translation3d.getX()), apply(translation3d.getY()), apply(translation3d.getZ()));
        }
        else{
            return translation3d;
        }
    }

    public static boolean shouldFlip(){
        return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    }
}
