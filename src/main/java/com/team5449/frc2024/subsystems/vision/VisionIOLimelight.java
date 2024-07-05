package com.team5449.frc2024.subsystems.vision;

import java.util.zip.CRC32C;

import com.team5449.lib.CConsole;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionIOLimelight implements VisionIO{
    private String name;
    private Transform3d cameraOffset;
    private double[] lastData = new double[6];

    private double pastTimestamp;

    public VisionIOLimelight(String name, Transform3d cameraOffset) {
        this.name = name;
        this.cameraOffset = cameraOffset;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        NetworkTableEntry botposeEntry;
        double[] fieldData;
        double[] botData;
        boolean hasShootTarget = false;
        int shootTarget = 0;

        botData = NetworkTableInstance.getDefault().getTable(name).getEntry("targetpose_robotspace").getDoubleArray(new double[7]);

        shootTarget = (int)NetworkTableInstance.getDefault().getTable(name).getEntry("tid").getInteger(0);

        // decides the pose based on the alliance
        // botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose");
        // if(shootTarget == 8 || shootTarget == 4) hasShootTarget = true;

        if (DriverStation.getAlliance().isEmpty()) {
            botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose");
        }
        else{
            //botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose");
            botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue");
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                //botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue");
                if(shootTarget == 4) hasShootTarget = true;
            } 
            else{
                //botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpired");
                if(shootTarget == 7) hasShootTarget = true;
            }
        }


        


        fieldData = botposeEntry.getDoubleArray(new double[7]);
        long updateTime = botposeEntry.getLastChange();
        Pose3d pose = new Pose3d(
            fieldData[0],
            fieldData[1],
            fieldData[2],
                new Rotation3d(
                        Math.toRadians(fieldData[3]),
                        Math.toRadians(fieldData[4]),
                        Math.toRadians(fieldData[5]))); // .transformBy(cameraOffset);

        // set if the Limelight has a target
        if (NetworkTableInstance.getDefault().getTable(name).getEntry("tv").getInteger(0) == 1) {
            inputs.hasTarget = true;
        } else {
            inputs.hasTarget = false;
        }

        // calculates total latency
        double latency = fieldData[6] / 1000;

        SmartDashboard.putBoolean("Vision/HasTarget", inputs.hasTarget);
        if (inputs.hasTarget) {
            inputs.timestamp = Timer.getFPGATimestamp() - latency;

            inputs.isNew = true;

            Pose2d pose2d = pose.toPose2d();

            inputs.x = pose2d.getX();
            inputs.y = pose2d.getY();
            inputs.rotation = pose2d.getRotation().getRadians();

            if(hasShootTarget){
                inputs.translationToTargetX = botData[2];
                
                inputs.rotationToTargetYaw = NetworkTableInstance.getDefault().getTable(name).getEntry("tx").getDouble(0.0);
            }
            
            //CConsole.stdout.log("hasShootTarget: ",hasShootTarget);
        } else {
            inputs.isNew = false;
        }

        lastData = fieldData;
    }

    @Override
    public String getName() {
        return name;
    }
}
