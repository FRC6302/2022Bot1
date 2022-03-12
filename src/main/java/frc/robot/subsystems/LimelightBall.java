// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightBall extends SubsystemBase {
  private static double x = 0, y = 0, targetFound = 0;
  private static double lastX = 0, lastY = 0;

  private String tableName = "limelightBall";

  //filtering balls
  private static int frameCount = 0;
  private static double previousX = 0;
  private static double previousY = 0;
  //private static double xTolerance = 1; //degrees
  //private static double yTolerance = 1; //degrees
  private static double distanceTolerance = 3;
  
  private static double frameCountMin = 3;
  private static ArrayList<Translation2d> ballPositionsList = new ArrayList<Translation2d>();


  /** Creates a new BallLimelight. */
  public LimelightBall() {
    try {
      //camera controls
      NetworkTableInstance.getDefault().getTable(tableName).getEntry("pipeline").setNumber(0);
      
      //changing camMode can be used to switch between the normal cam and the darkened targeting mode
      NetworkTableInstance.getDefault().getTable(tableName).getEntry("camMode").setNumber(0); //1 is light mode
      
      NetworkTableInstance.getDefault().getTable(tableName).getEntry("ledMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable(tableName).getEntry("stream").setNumber(0);
      }
      catch (RuntimeException ex){
        DriverStation.reportError("error setting limelight values because: " + ex.getMessage(), true);
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
    
    NetworkTableEntry tx = table.getEntry("tx"); 
    NetworkTableEntry ty = table.getEntry("ty"); 
    //NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    
    //read values periodically
    //maybe change default value to return the last x? Then you dont need lastX variable
    x = tx.getDouble(0.0); //ranges from -29.8 to 29.8 degrees for LL2
    y = ty.getDouble(0.0); //ranges from -24.85 to 24.85 degrees for LL2
    //double area = ta.getDouble(0.0); //ranges from 0 to 100% of image
    targetFound = tv.getDouble(0.0);

    if (getTargetFound()) {
      lastX = x;
      lastY = y;
    }
  }

  public static Translation2d getNearestBallTranslation(Pose2d robotPose) {
    double currX = getX();
    double currY = getY();
    ballPositionsList.add(new Translation2d(currX, currY));

    
    //if ball is close to where it was in the last frame, then it is probably the same ball
    /*if ((currX <= previousX + xTolerance || currX >= previousX - xTolerance) && (currY <= previousY + yTolerance || currY >= previousY - yTolerance)) {
      previousX = currX;
      previousY = currY;
      frameCount++;
    }
    else {
      frameCount = 0;
    }*/

    //make it so if the ball has been in at least 3 of the last 4 frames, then track it
    for (int i = 0; i < frameCountMin; i++) {
      Translation2d firstLocation = new Translation2d(ballPositionsList.get(0).getX(), ballPositionsList.get(0).getY());
      double ballDelta = ballPositionsList.get(i).getDistance(firstLocation);
      if (ballDelta < distanceTolerance) {
        //moves 2nd frame to 1st place in list to open up the end of list for the new frame
        ballPositionsList.set(i, ballPositionsList.get(i + 1));
      }
    }
    
    double ballDelta = Math.sqrt(Math.pow(currX - previousX, 2) + Math.pow(currY - previousY, 2));
    if (ballDelta < distanceTolerance) {
      previousX = currX;
      previousY = currY;
      frameCount++;
    }
    else {
      frameCount = 0;
    }

    double distance;
    //if the ball has been the same for 3 or more frames, then it's fine to track it
    if (frameCount > frameCountMin) {
      distance = getNearestBallDistance(robotPose.getRotation().getDegrees());
    }
    else {
      distance = 0;
    }

    return new Translation2d(robotPose.getX() + distance * Math.cos(Units.degreesToRadians(currX)),
      robotPose.getY() + distance * Math.sin(Units.degreesToRadians(currX)));

  }

  private static double getNearestBallDistance(double gyroAngle) {
    double rawDistance = Constants.limelightToBallCenterHeight / (Math.tan(Math.toRadians(lastY + Constants.limelightBallMountDegreeOffset))
      * Math.cos(Math.toRadians(lastX)));
    //TODO: get distance to robot center instead of to limelight
    return rawDistance;
  }

  public static double getX() {
    return x;
  }

  public static double getY() {
    return y;
  }

  public static double getLastX() {
    return lastX;
  }

  public static double getLastY() {
    return lastY;
  }

  public static boolean getTargetFound() {
    return targetFound == 1;
  }

  /*public static double getArea() {
    return area;
  }*/
}
