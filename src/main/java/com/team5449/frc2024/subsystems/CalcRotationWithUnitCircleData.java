package com.team5449.frc2024.subsystems;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.util.Units;

public class CalcRotationWithUnitCircleData{
    DoubleSupplier mX,mY;
    double lastrad=0;
    double trigv;
    public CalcRotationWithUnitCircleData(DoubleSupplier getX,DoubleSupplier getY,double InitalRotation,double ActivationBias)
    {
        mX=getX;
        mY=getY;
        lastrad=InitalRotation;
        if(ActivationBias>1 || ActivationBias<0)
        {
            throw new VerifyError("Verifing ActivationBias when constructing CalcRotationWithUnitCircleData out of range.");
        }
        trigv=ActivationBias;
    }
    public double calculate()
    {
        double x=mX.getAsDouble(),y=mY.getAsDouble();
        if(Math.sqrt(x*x+y*y)>=trigv)
        {
            lastrad = Math.atan2(y, x) - Units.degreesToRadians(90);
        }
        return lastrad;
    }
    public void reset()
    {
        lastrad = 0;
    }
}