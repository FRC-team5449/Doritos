package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.subsystems.intake.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax mIntake1;
  /** Creates a new Intake. */
  public Intake() {
    mIntake1 = new SparkMax(IntakeConstants.intakeCanId, MotorType.kBrushless);
    mIntake1.setInverted(IntakeConstants.intakeInversed);
  }

  public void setIntakeSpeed(double percent){
    mIntake1.set(percent);
  }

  @Override
  public void periodic() {
  }
}