package com.team5449.lib.util;

import java.util.function.BooleanSupplier;

import com.team5449.frc2024.Constants;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControllerUtil {
    private final GenericHID Controller;
    public ControllerUtil(GenericHID Controller){
        this.Controller=Controller;
    }
    public int GetButton()
    {
        return ControllerUtil.GetButton(Controller);
    }
    public static int GetXboxVal(String key){
        return 1<<(XboxController.Button.valueOf("k"+key).value-1);
    }
    public static int GetButton(GenericHID Controller)
    {
        int i,res=0;
        for(i=1;i<=16;i++){
            if(Controller.getRawButton(i)){
                res|=1<<(i-1);
            }
        }
        return res;
    }
    private static int btnstate,btnoldst=0,resbtnst=0;
    private static double ts=0,cs=0;
    public BooleanSupplier toCond(int cond)
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
            // SmartDashboard.putNumber("resbtnst",resbtnst);
            // SmartDashboard.putNumber("btnstate",btnstate);
            // SmartDashboard.putNumber("btnoldst",btnoldst);
            return cond==resbtnst;
        };
    }
}
