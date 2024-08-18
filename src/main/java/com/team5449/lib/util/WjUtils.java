package com.team5449.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WjUtils {
  public final static Translation2d applyDeadband(Translation2d translation, double deadband,double maxMagnitude){
    if(translation.getNorm()<=deadband){
        return new Translation2d(0, 0);
      }else{
      return translation.times((translation.getNorm()-deadband)*maxMagnitude/(maxMagnitude-deadband));
    }
  }
  public final static Translation2d applyDeadband(Translation2d translation, double deadband){
    return applyDeadband(translation, deadband,1);
  }
  public static final class JoystickInputHandler {
    //private Translation2d displacement=new Translation2d();
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter omegaLimiter;

    public JoystickInputHandler(double xSlewRate, double ySlweRate, double omegaSlewRate){
      xLimiter=new SlewRateLimiter(xSlewRate);
      yLimiter=new SlewRateLimiter(ySlweRate);
      omegaLimiter=new SlewRateLimiter(omegaSlewRate);
    }
    private final double calcOutput(double val){
      return Math.copySign(val*val, -val);
    }
    public final double getXInput(double xInput/*,double yInput*/){
      xInput=MathUtil.applyDeadband(xInput, 0.1);
      return calcOutput(xLimiter.calculate(xInput));
      //displacement=WjUtils.applyDeadband(new Translation2d(xInput, yInput),0.1);
      //displacement=new Translation2d(stickValueHandler(xLimiter.calculate(displacement.getX())), yLimiter.calculate(displacement.getY()));
      //return displacement.getX();
    }
    public final double getYInput(double yInput){
      yInput=MathUtil.applyDeadband(yInput, 0.1);
      return calcOutput(yLimiter.calculate(yInput));
      //return displacement.getY();
    }
    public final double getOmegaInput(double omegaInput){
      omegaInput=MathUtil.applyDeadband(omegaInput,0.1);
      return calcOutput(omegaLimiter.calculate(omegaInput));
    }
  }

  public static final double logOnSmartDashboard(String name, double data){
    SmartDashboard.putNumber(name, data);
    return data;
  }
}
