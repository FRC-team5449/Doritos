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

public class AutoMidCommand extends SequentialCommandGroup{
    private static final PathPlannerPath MidM1 = PathPlannerPath.fromPathFile("Mid2M1");
    private static final PathPlannerPath M1M2 = PathPlannerPath.fromPathFile("M12M2");
    private static final PathPlannerPath M2M3 = PathPlannerPath.fromPathFile("M22M3");
    private static final PathPlannerPath M3M4 = PathPlannerPath.fromPathFile("M32M4");
    private static final PathPlannerPath M4M5 = PathPlannerPath.fromPathFile("M42M5");
    private static final PathPlannerPath M1SHOOT = PathPlannerPath.fromPathFile("M12Q12Shoot");
    private static final PathPlannerPath M2SHOOT = PathPlannerPath.fromPathFile("M22Q12Shoot");
    private static final PathPlannerPath M3SHOOT = PathPlannerPath.fromPathFile("M32Q4Shoot");
    private static final PathPlannerPath M4SHOOT = PathPlannerPath.fromPathFile("M42Q4Shoot");
    private static final PathPlannerPath M5SHOOT = PathPlannerPath.fromPathFile("M52Q4Shoot");
    private static final PathPlannerPath[] MM = {MidM1, M1M2, M2M3, M3M4, M4M5};
    private static final PathPlannerPath[] MSHOOT = {M1SHOOT, M2SHOOT, M3SHOOT, M4SHOOT, M5SHOOT};
    private static final PathPlannerPath[] SHOOTM = {
        null,
        PathPlannerPath.fromPathFile("Q12Shoot2M2"), 
        PathPlannerPath.fromPathFile("Q12Shoot2M3"), 
        PathPlannerPath.fromPathFile("Q4Shoot2M4"), 
        PathPlannerPath.fromPathFile("Q4Shoot2M5")
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
    public AutoMidCommand(Intake i, Shooter s, ArmPoseCommand m, DrivetrainSubsystem mDrive){
        Consumer<Command> sequenceRun = (c) -> {
            Translation2d mRobot = mDrive.getPose().getTranslation().nearest(Arrays.asList(MSHOOT_poses));
            for(int j=0;j<MSHOOT_poses.length;j++){
                if(MSHOOT_poses[j]==mRobot){
                    mStartPose.getObject("traj").setPose(mRobot.getX(), mRobot.getY(), new Rotation2d());
                    AutoBuilder.followPath(MSHOOT[j]).andThen(new AutoShootCommand(new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 40, true), m, 2)).andThen(c).schedule();
                    NoteI = j;
                    break;
                }
            }
            
        };
        SmartDashboard.putData("Drive/StartPose", mStartPose);
        mStartPose.setRobotPose(MidM1.getPreviewStartingHolonomicPose());
        addCommands(new InstantCommand(() -> mDrive.resetPose(MidM1.getPreviewStartingHolonomicPose())));

        // addCommands(new InstantCommand(() -> m.setPose(ArmSystemState.SHOOTING)),new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 40));
        addCommands(new AutoShootCommand(new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 40, true), m, 2));
        addCommands(Commands.sequence(
            AutoBuilder.followPath(MidM1),
            // new InstantCommand(() -> NoteI = 1),
            AutoBuilder.followPath(M1M2),
            // new InstantCommand(() -> NoteI = 2),
            AutoBuilder.followPath(M2M3),
            // new InstantCommand(() -> NoteI = 3),
            AutoBuilder.followPath(M3M4),
            // new InstantCommand(() -> NoteI = 4),
            AutoBuilder.followPath(M4M5)
            // new InstantCommand(() -> NoteI = 5)
        ).raceWith(new IntakeCommand(s, i, m, false)));
        addCommands(
            new InstantCommand(() -> {
                if(NoteI == 5){
                    mDrive.setTargetVelocity(new ChassisSpeeds());
                    return;
                }
                SequentialCommandGroup mCmd = new SequentialCommandGroup(
                    AutoBuilder.followPath(SHOOTM[NoteI])
                    // new InstantCommand(() -> NoteI++),
                );
                NoteI+=2;
                for(int j=NoteI;j<5;j++){
                    mCmd.addCommands(AutoBuilder.followPath(MM[j]));
                }
                mCmd.raceWith(new IntakeCommand(s, i, m, false)).andThen(new InstantCommand(() -> sequenceRun.accept(Commands.none())));
                
                sequenceRun.accept(mCmd);
                // mCmd.schedule();
            })
        );
        
        // addCommands(AutoBuilder.followPath(MidM1));
        // addCommands(new PrintCommand("HELLO WORLD"));
    }
}