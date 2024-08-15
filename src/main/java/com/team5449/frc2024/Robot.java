// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024;


import org.littletonrobotics.junction.LoggedRobot;

import com.team5449.frc2024.subsystems.score.Arm;
//import com.team5449.lib.SaveLimelightPNG;
import com.team5449.lib.util.ControllerUtil;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
/*import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;*/
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //private SaveLimelightPNG mSave = new SaveLimelightPNG();

  @Override
  public void robotInit() {
    //m_robotContainer = new RobotContainer();

    //TODO Do not comment it
    /*UsbCamera camera = CameraServer.startAutomaticCapture();

    camera.setResolution(320, 240);*/

    //mSave.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if(DriverStation.getAlliance().isPresent()){
      SmartDashboard.putBoolean("Alliance Choice", DriverStation.getAlliance().get() != Alliance.Red);
    }

    SmartDashboard.putBoolean("Reload", m_robotContainer.conditionReload.getAsBoolean());
    SmartDashboard.putBoolean("OverShoot", m_robotContainer.conditionOverShoot.getAsBoolean());
    //SmartDashboard.putBoolean("Arm/AtSetPoint", Arm.getInstance().isArmAtSetpoint());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    //mSave.save("/home/lvuser/Images/path.png", null);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();


    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
