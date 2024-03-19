package com.team5449.frc2024.subsystems.vision;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Led extends SubsystemBase{

    private final PWMSparkMax mLed;

    public Led(){
        mLed = new PWMSparkMax(9);
    }
    public void setColor(Color kColor){
        mLed.set(kColor.motorPower);
    }

    private static int Color_LoadCount = 0;
    public enum Color{
        kRed(0.61),
        kHotPink(0.57),
        kDarkRed(0.59),
        kRedOrange(0.63),
        kOrange(0.65),
        kGold(0.67),
        kYellow(0.69),
        kLawnGreen(0.71),
        kLime(0.75),
        kGreen(0.77),
        kBlueGreen(0.79),
        kAqua(0.81),
        kSkyBlue(0.83),
        kDarkBlue(0.85),
        kBlue(0.87),
        kBlueViolet(0.89),
        kViolet(0.91),
        kWhite(0.93),
        kGray(0.95),
        kDarkGray(0.97),
        kBlack(0.99);


        public double motorPower;
        public String PrintString;

        private Color(double motorPower){
            this.motorPower = motorPower;
            java.lang.reflect.Field[] fields = this.getClass().getDeclaredFields();
            //TODO: unpredictable bug may caused by field's order doesn't match the initalized.
            PrintString = fields[Color_LoadCount].getName();
            Color_LoadCount++;
        }

        @Override
        public String toString()
        {
            return "Color."+PrintString+"(motorPower = "+motorPower+")";
        }
    }


    @Override
    public void periodic(){

    }
}