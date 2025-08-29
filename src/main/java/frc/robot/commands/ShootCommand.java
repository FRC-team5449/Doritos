package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends Command {
    private final Arm arm;
    private final Shooter shooter;
    private boolean isTransitRunning;
    private boolean isNoteOuted;

    private double shooterSetpoint;

    public ShootCommand(Arm arm, Shooter shooter, double shooterSetpoint) {
        this.arm = arm;
        this.shooter = shooter;
        this.shooterSetpoint = shooterSetpoint;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        isTransitRunning = false;
        isNoteOuted = false;
    }

    @Override
    public void execute() {
    shooter.setShooterRPM(shooterSetpoint);
    SmartDashboard.putBoolean("isShooterAtSetpoint", shooter.isShooterAtSetpoint());
    if(!shooter.isShooterAtSetpoint() && isTransitRunning){
        shooter.setTransitSpeed(0);
        isTransitRunning = false;
        isNoteOuted = true;
        System.out.println("Shoot!!!");
        }
    if(shooter.isShooterAtSetpoint()/* && isArmSet.getAsBoolean()*/ /*&& !isTransitRunning*/){
        shooter.setTransitSpeed(1);
        isTransitRunning = true;
    }
    else{
        shooter.setTransitSpeed(0);
        isTransitRunning = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterOpenloop(0);
        shooter.setTransitSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}