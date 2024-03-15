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
    private Constants.checkTarget colorCheck; 
    private final ColorSensorV3 m_ColorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);
    private final Color kHasTarget = new Color(255, 128, 0);
    private final Color kEmpty = new Color(0.150, 0.150, 0.150);
    private final ColorMatch m_ColorMatch = new ColorMatch();

    public ColorSensor()
    {
        m_ColorMatch.addColorMatch(kHasTarget);
        m_ColorMatch.addColorMatch(kEmpty);
    }

    public Constants.checkTarget[] getTarget()
    {
        return new Constants.checkTarget[]{colorCheck};
    }

    @Override
    public void periodic()
    {
        Color detectedTarget = m_ColorSensorV3.getColor();
        ColorMatchResult matchResult = m_ColorMatch.matchClosestColor(detectedTarget);
        if(matchResult.color == kHasTarget)
        {
            colorCheck = Constants.checkTarget.HASTARGET;
        }
        else
        {
            colorCheck = Constants.checkTarget.EMPTY;
        }

        SmartDashboard.putNumber("hasBall", colorCheck.ordinal());
    }
}
