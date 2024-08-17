// package com.team5449.frc2024.autos;

// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;

// public class PathTrajectories {
//     private PathConstraints constraints = new PathConstraints(3.5, 3, 100, 80);
    
//     public PathPlannerTrajectory testTrajectory;

//     public PathTrajectories(){
//         testTrajectory = getTrajectory("Example Path");
//     }

//     public PathPlannerTrajectory getTrajectory(String filename, ChassisSpeeds startSpeeds, Rotation2d startHeading){
//         PathPlannerPath path = PathPlannerPath.fromPathFile(filename);
//         return new PathPlannerTrajectory(path, startSpeeds, startHeading);
//     }

//     public PathPlannerTrajectory getTrajectory(String filename){
//         return getTrajectory(filename, new ChassisSpeeds(), new Rotation2d());
//     }
// }
