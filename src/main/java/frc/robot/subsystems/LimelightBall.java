// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  
  private static int frameCountMin = 3;
  private static int framesTracked = 4;
  //private static ArrayList<Translation2d> ballPositionsList = new ArrayList<Translation2d>();
  private static Translation2d[] ballPositionArray = new Translation2d[framesTracked];


  /** Creates a new BallLimelight. */
  public LimelightBall() {
    try {
      NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
      //camera controls
      table.getEntry("pipeline").setNumber(0);
      
      //changing camMode can be used to switch between the normal targeting mode and light mode for driving
      table.getEntry("camMode").setNumber(0); //1 is light mode, 0 is normal
      
      table.getEntry("ledMode").setNumber(0);
      table.getEntry("stream").setNumber(0);
    }
    catch (RuntimeException ex){
        DriverStation.reportError("error setting limelight values because: " + ex.getMessage(), true);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
    
    /*NetworkTableEntry tx = table.getEntry("tx"); 
    NetworkTableEntry ty = table.getEntry("ty"); 
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");*/
    
    //read values periodically
    //maybe change default value to return the last x? Then you dont need lastX variable
    x = table.getEntry("tx").getDouble(0); //ranges from -29.8 to 29.8 degrees for LL2
    y = table.getEntry("ty").getDouble(0); //ranges from -24.85 to 24.85 degrees for LL2
    double area = table.getEntry("ta").getDouble(0); //ranges from 0 to 100% of image
    targetFound = table.getEntry("tv").getDouble(0);

    if (getTargetFound()) {
      lastX = x;
      lastY = y;
    }
  }

  public static Pose2d getNearestBallPose(Pose2d robotPose) {
    double currX = getX();
    double currY = getY();
    //ballPositionsList.add(new Translation2d(currX, currY));
    frameCount = 0;

    //shifts all the array elements over by 1 to make room for the new measurement
    for (int i = 0; i < framesTracked - 1; i++) {
      ballPositionArray[i] = ballPositionArray[i + 1];
    }
    //adds the new measurement to end of array
    ballPositionArray[framesTracked] = new Translation2d(currX, currY);
    
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
    /*for (int i = 0; i < frameCountMin; i++) {
      Translation2d prevLocation = new Translation2d(ballPositionsList.get(i).getX(), ballPositionsList.get(i).getY());
      double ballDelta = ballPositionsList.get(i + 1).getDistance(prevLocation);
      if (ballDelta < distanceTolerance) {
        //moves 2nd frame to 1st place in list to open up the end of list for the new frame
        frameCount++;
      }
      //ballPositionsList.set(i, ballPositionsList.get(i + 1));
    }
    ballPositionsList.remove(framesTracked - 1);*/

    for (int i = 1; i < framesTracked; i++) {
      //gets the distance that the ball traveled inbetween frames. If the limelight is switching between targets then the distance will be high
      double ballDelta = ballPositionArray[i].getDistance(ballPositionArray[i - 1]);
      //if the target is close to where it was last frame, it can be assumed it is the same ball
      if (ballDelta < distanceTolerance) {
        frameCount++;
      }
    }
    


    /*double ballDelta = Math.sqrt(Math.pow(currX - previousX, 2) + Math.pow(currY - previousY, 2));
    if (ballDelta < distanceTolerance) {
      previousX = currX;
      previousY = currY;
      frameCount++;
    }
    else {
      frameCount = 0;
    }*/

    double distance;
    //if the ball has been the same for 3 or more frames, then it's fine to track it
    if (frameCount > frameCountMin) {
      distance = getNearestBallDistance(robotPose.getRotation().getDegrees());
    }
    else {
      distance = 0;
    }

    return new Pose2d(
      robotPose.getX() + distance * Math.cos(Units.degreesToRadians(currX)),
      robotPose.getY() + distance * Math.sin(Units.degreesToRadians(currX)),
      //base rotation of what way the robot should face in order to intake. Based on tx of ball
      new Rotation2d());

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
