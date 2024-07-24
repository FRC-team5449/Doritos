package com.team5449.frc2024.subsystems.vision;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.team5449.frc2024.Constants;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase
{
    // private Constants.checkTarget colorCheck; 
    // private boolean hasTarget;
    private final ColorSensorV3 m_ColorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);
    // private final Color kHasTarget = new Color(255, 128, 0);
    // private final Color kEmpty = new Color(0.150, 0.150, 0.150);
    // private final ColorMatch m_ColorMatch = new ColorMatch();

    public ColorSensor()
    {
        // m_ColorMatch.addColorMatch(kHasTarget);
        // m_ColorMatch.addColorMatch(kEmpty);
    }

    public boolean getTarget()
    {
        Color detectedTarget = m_ColorSensorV3.getColor();
        SmartDashboard.putNumberArray("Color", new double[]{detectedTarget.red, detectedTarget.green, detectedTarget.blue});
        return detectedTarget.red>0.35;
    }

    @Override
    public void periodic()
    {
        
        /*ColorMatchResult matchResult = m_ColorMatch.matchClosestColor(detectedTarget);
        if(matchResult.color == kHasTarget)
        {
            colorCheck = Constants.checkTarget.HASTARGET;
        }
        else
        {
            colorCheck = Constants.checkTarget.EMPTY;
        }*/

        //SmartDashboard.putNumber("hasBall", colorCheck.ordinal());
    }
}
