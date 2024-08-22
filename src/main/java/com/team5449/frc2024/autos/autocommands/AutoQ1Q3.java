package com.team5449.frc2024.autos.autocommands;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team5449.frc2024.RobotContainer;
import com.team5449.frc2024.commands.ArmPoseCommand;
import com.team5449.frc2024.commands.IntakeCommand;
import com.team5449.frc2024.commands.ShootCommand;
import com.team5449.frc2024.commands.ArmPoseCommand.ArmSystemState;
import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;
import com.team5449.frc2024.subsystems.score.Intake;
import com.team5449.frc2024.subsystems.score.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoQ1Q3 extends SequentialCommandGroup{
    private static final PathPlannerPath UpQ1 = PathPlannerPath.fromPathFile("Up2Q1");
    private static final PathPlannerPath Q1MidShoot = PathPlannerPath.fromPathFile("Q12MidShoot");
    private static final PathPlannerPath MidShootQ3 = PathPlannerPath.fromPathFile("MidShoot2Q3");
    private static final PathPlannerPath Q3DownShoot = PathPlannerPath.fromPathFile("Q32DownShoot");
    private static final PathPlannerPath Q1Q3 = PathPlannerPath.fromPathFile("Q12Q3");



    private static final PathPlannerPath StartPoseChooser = UpQ1;

    private static final PathPlannerPath[] MM = {StartPoseChooser, Q1Q3};
    private static final PathPlannerPath[] MSHOOT = {Q1MidShoot, Q3DownShoot};
    private static final PathPlannerPath[] SHOOTM = {
        null,
        MidShootQ3, 
    };
    private static final Translation2d[] MSHOOT_poses;
    static{
        MSHOOT_poses=new Translation2d[MSHOOT.length];
        for(int i=0;i<MSHOOT.length; i++){
            MSHOOT_poses[i] = MSHOOT[i].getPreviewStartingHolonomicPose().getTranslation();
        }
    }
    // private static final PathPlannerPath[] SHOOTM = {M2SHOOT.flipPath(), M3SHOOT.flipPath(), , M4SHOOT, M5SHOOT};
    private static final Field2d mStartPose = new Field2d();
    private int NoteI = 0;
    public AutoQ1Q3(Intake i, Shooter s, ArmPoseCommand m, DrivetrainSubsystem mDrive){
        // Consumer<Command> sequenceRun = (c) -> {
        //     Translation2d mRobot = mDrive.getPose().getTranslation().nearest(Arrays.asList(MSHOOT_poses));
        //     for(int j=0;j<MSHOOT_poses.length;j++){
        //         if(MSHOOT_poses[j]==mRobot){
        //             mStartPose.getObject("traj").setPose(mRobot.getX(), mRobot.getY(), new Rotation2d());
        //             System.out.println("SHOOT POSE "+j);
        //             (AutoBuilder.followPath(MSHOOT[j]).andThen(new AutoShootCommand(new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 60, true), m, 2)).andThen(Commands.print("Ten")).andThen(c)).schedule();
        //             NoteI = j;
        //             break;
        //         }
        //     }
            
        // };

        SmartDashboard.putData("Drive/StartPose", mStartPose);
        mStartPose.setRobotPose(StartPoseChooser.getPreviewStartingHolonomicPose());
        addCommands(new InstantCommand(() -> mDrive.resetPose(StartPoseChooser.getPreviewStartingHolonomicPose())));

        // addCommands(new InstantCommand(() -> m.setPose(ArmSystemState.SHOOTING)),new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 40));
        addCommands(new AutoShootCommand(new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 40, true), m, 2));
        addCommands(Commands.sequence(
            AutoBuilder.followPath(StartPoseChooser),
            new InstantCommand(() -> NoteI = 1),
            AutoBuilder.followPath(Q1Q3),
            new InstantCommand(() -> NoteI = 2),
            Commands.waitSeconds(1),
            new InstantCommand(() -> NoteI = 3)
        ).raceWith(new IntakeCommand(s, i, m, false)));
        addCommands(
            new InstantCommand(() -> {
                if(NoteI == 3){
                    mDrive.setTargetVelocity(new ChassisSpeeds());
                    return;
                }
                if(NoteI == 0){
                    AutoBuilder.followPath(Q1MidShoot).andThen(new AutoShootCommand(new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 40, true), m, 2)).andThen(
                        AutoBuilder.followPath(MidShootQ3).raceWith((new IntakeCommand(s, i, m, false))).andThen(AutoBuilder.followPath(Q3DownShoot).andThen(new AutoShootCommand(new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 40, true), m, 2)))
                    ).schedule();
                }
                if(NoteI == 1){
                    AutoBuilder.followPath(Q3DownShoot).andThen(new AutoShootCommand(new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 40, true), m, 2)).schedule();
                }
                // SequentialCommandGroup mCmd = new SequentialCommandGroup(
                //     AutoBuilder.followPath(SHOOTM[NoteI])
                //     // new InstantCommand(() -> NoteI++),
                // );
                // // NoteI+=2;
                // // for(int j=NoteI;j<5;j++){
                // //     mCmd.addCommands(AutoBuilder.followPath(MM[j]));
                // // }
                
                // // sequenceRun.accept(mCmd.raceWith(new IntakeCommand(s, i, m, false)).andThen(new InstantCommand(() -> sequenceRun.accept(Commands.none()))));
                // // sequenceRun.accept(mCmd);
                // sequenceRun.accept(Commands.none());
                // // mCmd.schedule();
            })
        );
        
        // addCommands(AutoBuilder.followPath(MidM1));
        // addCommands(new PrintCommand("HELLO WORLD"));
    }
}