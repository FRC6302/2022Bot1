// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
  //change the I2C port below to match the connection of your color sensor to the rio
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch colorMatcher = new ColorMatch();

  //this line should work but it doesnt - rev library is messed up or something
  //private final Color blue = ColorMatch.makeColor(0.143, 0.427, 0.429);

  private final Color blue = new Color(0.143, 0.427, 0.429);
  private final Color green = new Color(0.197, 0.561, 0.240);
  private final Color red = new Color(0.561, 0.232, 0.114);
  private final Color yellow = new Color(0.361, 0.524, 0.113);



  /** Creates a new ColorSensor. */
  public ColorSensor() {
    colorMatcher.addColorMatch(blue);
    colorMatcher.addColorMatch(green);
    colorMatcher.addColorMatch(red);
    colorMatcher.addColorMatch(yellow);   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    Color detectedColor = colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == blue) {
      colorString = "Blue";
    } else if (match.color == red) {
      colorString = "Red";
    } else if (match.color == green) {
      colorString = "Green";
    } else if (match.color == yellow) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
}
