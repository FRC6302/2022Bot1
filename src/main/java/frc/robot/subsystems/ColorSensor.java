// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {

  //change the I2C port below to match the connection of your color sensor to the rio
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);

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

  private final Color blue = new Color(0.20, 0.44, 0.38); //Color(0.143, 0.427, 0.429);
  private final Color green = new Color(0.197, 0.561, 0.240);
  private final Color red = new Color(0.50, 0.35, 0.12); //Color(0.561, 0.232, 0.114);
  private final Color yellow = new Color(0.361, 0.524, 0.113);

  private final DriverStation.Alliance alliance;
  private final Color allianceColor;
  
  private static double ballsPickedUp;
  private static String latestBall = "unknown";
  private static String currentBall = "unknown";
  private static String allianceColorStr = "unknown";

  //tracks the time its been since we last saw the opposing team's ball
  private static Timer lastOppositeBallTimer = new Timer();
  //how long have we been seeing the opposing ball color
  private static Timer currentOppositeBallTimer = new Timer();


  /** Creates a new ColorSensor. */
  public ColorSensor() {
    colorMatcher.addColorMatch(blue);
    colorMatcher.addColorMatch(green);
    colorMatcher.addColorMatch(red);
    colorMatcher.addColorMatch(yellow);

    //how confident the sensor has to be in order to say something is a color
    colorMatcher.setConfidenceThreshold(0.95);

    alliance = DriverStation.getAlliance();
    if (alliance == DriverStation.Alliance.Blue) {
      allianceColor = blue;
      allianceColorStr = "blue";
    }
    else if (alliance == DriverStation.Alliance.Red) {
      allianceColor = red;
      allianceColorStr = "red";
    }
    else {
      DriverStation.reportWarning("COULD NOT GET ALLIANCE COLOR, DEFAULTING TO YELLOW", false);
      allianceColor = blue;
      allianceColorStr = "blue";
    }
    SmartDashboard.putString("alliance", allianceColorStr);

    lastOppositeBallTimer.start();
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
    String colorString = "";
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    /*if (match.color == allianceColor) {
      ballsPickedUp++;
    }*/
    
    if (match.color == blue) {
      colorString = "blue";
      latestBall = "blue";
      currentBall = "blue";
      ballsPickedUp++;
    } else if (match.color == red) {
      colorString = "red";
      latestBall = "red";
      currentBall = "red";
      ballsPickedUp++;
    /*} else if (match.color == green) {
      colorString = "Green";
    } else if (match.color == yellow) {
      colorString = "Yellow";*/
    } else {
      colorString = "unknown";
      currentBall = "unknown";
      //i dont update lastestBall here because the whole point of the variable is to have the lastest ball
    }

    //if we are seeing our ball or no ball, reset the timer
    if (!getLatestBallIsAlliance() && !getCurrentBallIsUnknown()) {
      lastOppositeBallTimer.reset();
    }

    if (getCurrentBallIsAlliance()) {
      currentOppositeBallTimer.reset();
    }
    else {
      currentOppositeBallTimer.start(); //doesnt effect the timer if its already running
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    SmartDashboard.putNumber("time seeing opposite ball", getTimeSeeingOppositeBall());
    SmartDashboard.putNumber("time since seen opposite ball", getTimeSinceOppositeBall());

    
  }

  public static double getBallsPickedUp() {
    return ballsPickedUp;
  }

  public static boolean getLatestBallIsAlliance() {
    return latestBall == allianceColorStr;
  }

  public static boolean getCurrentBallIsAlliance() {
    if (currentBall == "unknown") {
      /*defaults to true if we dont see a ball so that everything is ready for when we do get a ball. We're more likely to get 
      our color next than the other color so everything should already be in place*/
      return true;
    }
    return currentBall == allianceColorStr;
  }

  public static boolean getCurrentBallIsUnknown() {
    return currentBall == "unknown";
  }

  public static double getTimeSinceOppositeBall() {
    return lastOppositeBallTimer.get();

  }

  public static double getTimeSeeingOppositeBall() {
    return currentOppositeBallTimer.get();
  }
}
