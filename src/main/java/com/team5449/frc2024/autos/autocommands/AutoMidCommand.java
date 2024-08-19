package com.team5449.frc2024.autos.autocommands;

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

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static final PathPlannerPath MM = {MidM1, M1M2, M2M3, M3M4, M4M5};
    private static final PathPlannerPath[] MSHOOT = {M1SHOOT, M2SHOOT, M3SHOOT, M4SHOOT, M5SHOOT};
    // private static final PathPlannerPath[] SHOOTM = {M2SHOOT.flipPath(), M3SHOOT.flipPath(), , M4SHOOT, M5SHOOT};
    private static final Field2d mStartPose = new Field2d();
    private int NoteI = 0;
    public AutoMidCommand(Intake i, Shooter s, ArmPoseCommand m, DrivetrainSubsystem mDrive){

        SmartDashboard.putData("Drive/StartPose", mStartPose);
        mStartPose.setRobotPose(MidM1.getPreviewStartingHolonomicPose());
        addCommands(new InstantCommand(() -> mDrive.resetPose(MidM1.getPreviewStartingHolonomicPose())));

        // addCommands(new InstantCommand(() -> m.setPose(ArmSystemState.SHOOTING)),new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 40));
        addCommands(new AutoShootCommand(new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 40, true), m, 2));
        addCommands(Commands.sequence(
            AutoBuilder.followPath(MidM1),
            new InstantCommand(() -> NoteI = 1),
            AutoBuilder.followPath(M1M2),
            new InstantCommand(() -> NoteI = 2),
            AutoBuilder.followPath(M2M3),
            new InstantCommand(() -> NoteI = 3),
            AutoBuilder.followPath(M3M4),
            new InstantCommand(() -> NoteI = 4),
            AutoBuilder.followPath(M4M5),
            new InstantCommand(() -> NoteI = 5)
        ).raceWith(new IntakeCommand(s, i, m, false)));
        addCommands(
            new InstantCommand(() -> {
                if(NoteI == 5){
                    mDrive.setTargetVelocity(new ChassisSpeeds());
                    return;
                }
                Commands.sequence(
                    AutoBuilder.followPath(MSHOOT[NoteI]),
                    new AutoShootCommand(new ShootCommand(s, m, ()-> m.getArmState()==ArmSystemState.SHOOTING, 40), m, 2),
                    new InstantCommand(() -> {NoteI++;AutoBuilder.followPath(MSHOOT[NoteI].flipPath()).schedule();}),
                    ,
                    new InstantCommand(() -> NoteI++),
                    AutoBuilder.followPath(MM[NoteI])
                ).schedule();
            })
        );
        
        // addCommands(AutoBuilder.followPath(MidM1));
        // addCommands(new PrintCommand("HELLO WORLD"));
    }
}