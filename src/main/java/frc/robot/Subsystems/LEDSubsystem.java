// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private Spark LEDController = new Spark(LEDConstants.LEDPort);
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
  }
  public void setToGold() {
    LEDController.set(0.33);
  }

  public void setToGreen() {
    LEDController.set(0.89);
  }

  public void setRainbow() {
    LEDController.set(-0.99); // 0.89 is green
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
