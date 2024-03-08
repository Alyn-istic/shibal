// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.LEDconstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

      // PWM port 9
      // Must be a PWM header, not MXP or DIO
      AddressableLED m_led = new AddressableLED(8);
  
      // Reuse buffer
      // Default to a length of 60, start empty output
      // Length is expensive to set, so only set it once, then just update data
      AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(149);

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

      m_led.setLength(m_ledBuffer.getLength());
  
      // Set the data
      m_led.setData(m_ledBuffer);
      m_led.start();
    

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for green
      m_ledBuffer.setRGB(i, 0, 255, 0);
   }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(
          new WaitCommand(AutonomousConstants.waitBeforeExecRoutine).andThen(m_autonomousCommand)
      );
     for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for gold
      m_ledBuffer.setRGB(i, 255, 215, 0);

      // if(DriverStation.getAlliance().get() == Alliance.Blue){
      //   for(var i = 0; i < m_ledBuffer.getLength(); i++){
      //   m_ledBuffer.setRGB(i, 0, 0, 255);
      //   }
      // }
      // else{
      //   for(var i = 0; i < m_ledBuffer.getLength(); i++){
      //   m_ledBuffer.setRGB(i, 255, 0, 0);
      //   }
      // }
     }
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
  public void teleopPeriodic(){ 
  if(SmartDashboard.getNumber("Intake/Shooter Motor 1 Speed", 0) > 0 || 
    SmartDashboard.getNumber("Intake/Shooter Motor 2 Speed", 0) > 0){
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for gold
      m_ledBuffer.setRGB(i, 255, 215, 0);
      }
    }
    else{
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for green
      m_ledBuffer.setRGB(i, 0, 255, 0);
      }
    }
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


