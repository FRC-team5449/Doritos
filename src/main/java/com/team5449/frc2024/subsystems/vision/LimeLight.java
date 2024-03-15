package com.team5449.frc2024.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase{
      /** Creates a new Limelight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  //XboxController Controller = new XboxController(0);
  private double x;
  private double y;
  private double area;
  private double[] position3D;
  // public void writePeriodicOutputs() {
  //     if(Controller.getButton(Button.A)){
  //         Limelight.enabled();
  //       }
  // }
  @Override
  public void periodic(){
    double rawPos[]=getPosition3D();
    SmartDashboard.putNumber("X", rawPos[0]);
    SmartDashboard.putNumber("Y", rawPos[1]);
    SmartDashboard.putNumber("Z", rawPos[2]);
  }
  public void enabled() {
      table.getEntry("ledMode").setNumber(1);
  }
  public double checkAprilTag(){
    return table.getEntry("tv").getDouble(0);
  }

  public double[] getPosition3D(){
      //table.getEntry()
      table.getEntry("camMode").setNumber(0);
      table.getEntry("stream").setNumber(0);
      table.getEntry("pipeline").setNumber(0);
      table.getEntry("snapshot").setNumber(0);
      table.getEntry("3d").setNumber(1);
      //mLimelight.enabled();
      double[] p = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
      for(int i=0; i<3; i++){
        p[i] = p[i]*2*3.2808398950131;
      }
      return p;
    }

      

}
