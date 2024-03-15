package com.team5449.frc2024.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team5449.frc2024.Constants;
import com.team5449.frc2024.RobotContainer;
import com.team5449.frc2024.autos.autocommands.DriveTrajectoryCommand;
import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PathChooser {

    private enum AutoMode{
        DriveStraightLine,
        DoNothing
    }

    private final PathTrajectories trajectories;

    private static final SendableChooser<AutoMode> autoChooser = new SendableChooser<>();

    private static final HolonomicPathFollowerConfig holoConfig = new HolonomicPathFollowerConfig(
        2.5, 
        Constants.driveBaseRadius,
        new ReplanningConfig(true, true) );

    public PathChooser(PathTrajectories pathTrajectories){
        trajectories = pathTrajectories;
        autoChooser.setDefaultOption("Do Nothing", AutoMode.DoNothing);
        autoChooser.addOption("Go Straight Line", AutoMode.DriveStraightLine);

        SmartDashboard.putData("AutoMode Option", autoChooser);
    }

    public Command getDoNothing(){
        return new InstantCommand(() -> System.out.println("Do Nothing"));
    }

    public Command getDriveStraight(RobotContainer container){
        return resetDrivePose(trajectories.testTrajectory, container)
            .andThen(follow(container, trajectories.testTrajectory));
    }

    public Command follow(RobotContainer container, PathPlannerTrajectory trajectory){
        return new DriveTrajectoryCommand(null, trajectory);
    }

    public Command getCommand(RobotContainer container){
        switch (autoChooser.getSelected()) {
            case DoNothing:
                return getDoNothing();
            case DriveStraightLine:
                return getDriveStraight(container);
            default:
                return getDoNothing();
        }
    }

    public Command driveStriaghtByBuilder(RobotContainer container){
        DrivetrainSubsystem drive = null;
        AutoBuilder.configureHolonomic(
            drive::getPose,
            drive::resetPose,
            drive::getMeasuredVelocity,
            drive::setTargetVelocity,
            holoConfig,
            () -> false,
            drive);
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path.path"));
    }

    public Command resetDrivePose(PathPlannerTrajectory trajectory, RobotContainer container){
        return new InstantCommand(() -> resetPose(
            new Pose2d(trajectory.getInitialState().positionMeters, trajectory.getInitialState().heading), container));
    }


    public void resetPose(Pose2d pose, RobotContainer container) {
        //container.getDrivetrainSubsystem().resetPose(pose);
    }

}
