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
    //maybe change default value to return the last x? Then you dont need lastX variable
    Limelight.x = tx.getDouble(0.0); //ranges from -29.8 to 29.8 degrees for LL2
    Limelight.y = ty.getDouble(0.0); //ranges from -24.85 to 24.85 degrees for LL2
    double area = ta.getDouble(0.0); //ranges from 0 to 100% of image
    Limelight.targetFound = tv.getDouble(0.0);

    if (getTargetFound()) {
      lastX = x;
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

  
  public static double getTargetDistanceSimple() {
    /*This is the formula presented in the limelight documentation and can be verified with simple trig.
    However, this only works if the LL is pointed right at the target (x=0). If it isnt, the distance wont be
    accurate due to how the camera works. See other method for more explanation.
    Limelight docs: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
    */
    distance = Constants.targetDeltaY / Math.tan(Math.toRadians(lastY + Constants.limelightMountDegreeOffset));
    
    SmartDashboard.putNumber("target distance simple", distance);
    return distance;

  }

  //TODO test the differences here

  public static double getTargetDistance() {
    /*Why the simple method above doesn't always work: 
    https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/7?u=frc6302
    and see here for the formula I used:
    https://www.chiefdelphi.com/t/calculating-distance-to-vision-target/387183/6?u=frc6302
    */
    distance = 0.381 / (Math.tan(Math.toRadians(lastY)) * Math.cos(Math.toRadians(lastX)));

    SmartDashboard.putNumber("target distance", distance);
    return distance;
  }
}
