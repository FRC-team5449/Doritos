package com.team5449.frc2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    private final Pose3d cameraOffest;
    private Pose3d pose;
    //XboxController Controller = new XboxController(0);
    // private double x;
    // private double y;
    // private double area;
    // private double[] position3D;
    // public void writePeriodicOutputs() {
    //     if(Controller.getButton(Button.A)){
    //         Limelight.enabled();
    //     }
    // }
    public LimeLight(Pose3d cameraOffest){
        this.cameraOffest = cameraOffest;
        pose = new Pose3d();
    }
    @Override
    public void periodic(){
        Pose3d rawPos=getPose3DRaw();
        SmartDashboard.putNumber("limelight/X", rawPos.getX());
        SmartDashboard.putNumber("limelight/Y", rawPos.getY());
        SmartDashboard.putNumber("limelight/Z", rawPos.getZ());
    }
    public void enabled() {
        table.getEntry("ledMode").setNumber(1);
    }
    public double checkAprilTag(){
        return table.getEntry("tv").getDouble(0);
    }

    public Pose3d getPose3DRaw(){
        //table.getEntry()
        table.getEntry("camMode").setNumber(0);
        table.getEntry("stream").setNumber(0);
        table.getEntry("pipeline").setNumber(0);
        table.getEntry("snapshot").setNumber(0);
        table.getEntry("3d").setNumber(1);
        //mLimelight.enabled();
        if(checkAprilTag()!=0){
            double[] p = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
            pose = new Pose3d(p[0], p[1], p[2], new Rotation3d(
                Math.toRadians(p[5]),
                Math.toRadians(p[3]),
                Math.toRadians(p[4])));
        }
        /*for(int i=0; i<3; i++){
            p[i] = p[i]*2*3.2808398950131;
        }*/
        return pose;
    }
    public Pose3d getPose3DBot(){
        Pose3d p=getPose3DRaw().rotateBy(cameraOffest.getRotation().unaryMinus());
        return new Pose3d(p.getTranslation().minus(cameraOffest.getTranslation()), p.getRotation());
    }
}
