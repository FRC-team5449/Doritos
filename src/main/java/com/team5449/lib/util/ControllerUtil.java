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
    public static long GetXboxVal(int port, String key, int CompMethod){
        return (1L<<(XboxController.Button.valueOf("k"+key).value-1))|(((long)CompMethod)<<40)|(((long)port&0xF)<<32);
    }
    public static long GetXboxVal(int port, String key){
        return GetXboxVal(port, key, 1);
    }
    private static int nowport=0;
    public static long GetXboxVal(String key, int CompMethod){
        return GetXboxVal(nowport, key, CompMethod);
    }
    public static long GetXboxVal(String key){
        return GetXboxVal(key, 1);
    }
    public static void setControlPort(int port){
        nowport=port;
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
    private static class Status{
        public int btnstate,btnoldst=0,resbtnst=0;
        public double ts=0,cs=0;
    }
    static Status st[]=new Status[DriverStation.kJoystickPorts];
    static{
        for(int i=0;i<DriverStation.kJoystickPorts;i++){
            st[i]=new Status();
        }
    }
    public static BooleanSupplier toCond(long cond)
    {
        return () -> {
            int port=((int)(cond>>32))&0xF;
            if(Timer.getFPGATimestamp()-st[port].ts>Constants.kLooperDt){
                st[port].btnstate=GetButton(port);
                if(st[port].btnoldst!=st[port].btnstate){
                    st[port].btnoldst=st[port].btnstate;
                    st[port].cs=Timer.getFPGATimestamp();
                }else{
                    if(Timer.getFPGATimestamp()-st[port].cs>Constants.ControlTimeout){
                        st[port].resbtnst=st[port].btnstate;
                    }
                }
            }
            SmartDashboard.putNumber("ControllerUtil/resbtnst/"+String.valueOf(port),st[port].resbtnst);
            SmartDashboard.putNumber("ControllerUtil/btnstate/"+String.valueOf(port),st[port].btnstate);
            SmartDashboard.putNumber("ControllerUtil/btnoldst/"+String.valueOf(port),st[port].btnoldst);
            switch ((int)(cond>>40)) {
                case 0:
                    return (((int)cond)&(st[port].resbtnst))!=0;
                case 1:
                    return ((int)cond)==st[port].resbtnst;
                default:
                    return false;
            }
            
        };
    }
}
