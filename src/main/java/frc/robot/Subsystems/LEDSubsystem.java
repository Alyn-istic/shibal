// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.LEDconstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private Spark LEDController = new Spark(LEDconstants.LEDTalonPort);
  
  public void setPresetGold(){
    LEDController.set(0.33);

  }

  public void setPresetGreen(){
    LEDController.set(0.89);
  }

  public void setRainbow(){
    LEDController.set(-0.99);
  }

  public void turnOff() {
    LEDController.set(0);
  }

  public DriverStation.Alliance getAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get();
    }
    return DriverStation.Alliance.Blue;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
