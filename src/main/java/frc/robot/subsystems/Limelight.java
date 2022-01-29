// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//subsystem for the limelight camera on our robot. Used for vision tracking and stuff
public class Limelight extends SubsystemBase {
  private static double x = 0, y = 0, area = 0, targetFound = 0;
  private static double lastX = 0, lastY = 0;

  private static double distance; //to target
  
  /**
   * Creates a new Limelight.
   */
  public Limelight() {
    try {
      //camera controls
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      
      //changing camMode can be used to switch between the normal cam and the darkened targeting mode
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //1 is light mode
      
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
      }
      catch (RuntimeException ex){
        DriverStation.reportError("error setting limelight values because: " + ex.getMessage(), true);
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    NetworkTableEntry tx = table.getEntry("tx"); 
    NetworkTableEntry ty = table.getEntry("ty"); 
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    
    //read values periodically
    Limelight.x = tx.getDouble(0.0); //ranges from -29.8 to 29.8 degrees for LL2
    Limelight.y = ty.getDouble(0.0); //ranges from -24.85 to 24.85 degrees for LL2
    double area = ta.getDouble(0.0); //ranges from 0 to 100% of image
    Limelight.targetFound = tv.getDouble(0.0);

    if (x != 0) {
      lastX = x;
    }
    if (y != 0) {
      lastY = y;
    }

    //posts to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area); 
    SmartDashboard.putNumber("LimelightTargetFound", targetFound);
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

  public static double getArea() {
    return area;
  }

  public static double getTargetDistance() {
    //figure out the formula based on linear regresion of data table with
    //x as area and y as the distance measured by me. Might be polynomial.
    //Formula will change based on size of reflective target/tape.
    //Might be able to work out formula with math and the camera's field of view?
    //return area * 5;


    //ty = Limelight.getY();
    distance = Constants.targetDeltaY / Math.tan(Math.toRadians(y + Constants.limelightMountDegreeOffset));
    SmartDashboard.putNumber("target distance", distance);
    return distance;

    //TODO: test this and then delete DistanceToTarget command
  }
}
