// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TouchSensor extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private I2C i2c = new I2C(i2cPort, 0);

  byte[] byteArray = {5};
  
  
  /** Creates a new TouchSensor. */
  public TouchSensor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    i2c.read(0, 1, byteArray);
    SmartDashboard.putNumber("touch sensor", byteArray[0]);

  }
}
