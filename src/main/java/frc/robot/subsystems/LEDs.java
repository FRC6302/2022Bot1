// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//UNTESTED
public class LEDs extends SubsystemBase {
  AddressableLED addressableLED = new AddressableLED(0);

  AddressableLEDBuffer buffer = new AddressableLEDBuffer(5);
  /** Creates a new LEDs. */
  public LEDs() {
    addressableLED.setLength(buffer.getLength());

    addressableLED.setData(buffer);

    addressableLED.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      buffer.setRGB(i, 255, 0, 0);
   }
   
   addressableLED.setData(buffer);
  }
}
