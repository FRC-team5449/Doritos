package com.team5449.lib.util;

import com.team5449.frc2024.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Util {
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, Constants.kEpsilon);
    }

    public static boolean chassisEpsilonEquals(ChassisSpeeds a, ChassisSpeeds b, double epsilon) {
        return Util.epsilonEquals(a.vxMetersPerSecond, b.vxMetersPerSecond, epsilon) &&
                Util.epsilonEquals(a.vyMetersPerSecond, b.vyMetersPerSecond, epsilon) &&
                Util.epsilonEquals(a.omegaRadiansPerSecond, b.omegaRadiansPerSecond, epsilon);
    }

    public static double limit(double max, double min, double val){
        return Math.max(Math.min(max, val), min);
    }

    /**
     * Mapping the value to the specified range [l,r), cycles when out of range
     * 
     * Useful for mapping rotations to [0,360)
     * @param l The left bound
     * @param r The right bound
     * @param val The value
     */
    public static double map(double l, double r, double val){
        double coeff=Math.floor(val/(r-l));
        return val-coeff*(r-l)+l;
    }
}
