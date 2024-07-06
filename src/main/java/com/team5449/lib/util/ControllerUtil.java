package com.team5449.lib.util;

import java.util.function.BooleanSupplier;

import com.team5449.frc2024.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControllerUtil {
    private final int ControllerPort;
    public ControllerUtil(GenericHID Controller){
        this.ControllerPort=Controller.getPort();
    }
    public int GetButton()
    {
        return ControllerUtil.GetButton(ControllerPort);
    }
    public static long GetXboxVal(String key, int CompMethod){
        return (1L<<(XboxController.Button.valueOf("k"+key).value-1))|(((long)CompMethod)<<32);
    }
    public static long GetXboxVal(String key){
        return GetXboxVal(key, 1);
    }
    public static int GetButtonCnt(int ControllerPort){
        return DriverStation.getStickButtonCount(ControllerPort);
    }
    public static int GetButton(int ControllerPort)
    {
        /*int i,res=0;
        for(i=1;i<=16;i++){
            if(Controller.getRawButton(i)){
                res|=1<<(i-1);
            }
        }
        return res;*/
        return DriverStation.getStickButtons(ControllerPort);
    }
    private int btnstate,btnoldst=0,resbtnst=0;
    private double ts=0,cs=0;
    public BooleanSupplier toCond(long cond)
    {
        return () -> {
            if(Timer.getFPGATimestamp()-ts>Constants.kLooperDt){
                btnstate=GetButton();
                if(btnoldst!=btnstate){
                    btnoldst=btnstate;
                    cs=Timer.getFPGATimestamp();
                }else{
                    if(Timer.getFPGATimestamp()-cs>Constants.ControlTimeout){
                        resbtnst=btnstate;
                    }
                }
            }
            SmartDashboard.putNumber("ControllerUtil/resbtnst/"+String.valueOf(ControllerPort),resbtnst);
            SmartDashboard.putNumber("ControllerUtil/btnstate/"+String.valueOf(ControllerPort),btnstate);
            SmartDashboard.putNumber("ControllerUtil/btnoldst/"+String.valueOf(ControllerPort),btnoldst);
            switch ((int)(cond>>32)) {
                case 0:
                    return (((int)cond)&resbtnst)!=0;
                case 1:
                    return ((int)cond)==resbtnst;
                default:
                    return false;
            }
            
        };
    }
}
