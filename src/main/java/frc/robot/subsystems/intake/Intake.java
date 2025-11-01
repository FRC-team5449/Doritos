package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.subsystems.intake.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax mIntake1;
  private static SparkBaseConfig intakeConfig;

  /** Creates a new Intake. */
  public Intake() {
    mIntake1 = new SparkMax(IntakeConstants.intakeCanId, MotorType.kBrushless);
    // mIntake1.setInverted(IntakeConstants.intakeInversed);
    intakeConfig.inverted(true);
    mIntake1.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setIntakeSpeed(double percent){
    mIntake1.set(percent);
  }

  @Override
  public void periodic() {
  }
}