// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//subsystem for the limelight camera on our robot. Used for vision tracking and stuff
public class LimelightGoal extends SubsystemBase {
  private static double x = 0, y = 0, area = 0, targetFound = 0;
  private static double lastX = 0, lastY = 0;

  private NetworkTableEntry xEntry, yEntry, targetFoundEntry;
  private String tableName = "limelight";

  
  /**
   * Creates a new Limelight.
   */
  public LimelightGoal() {
    try {
      NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
      //camera controls
      table.getEntry("pipeline").setNumber(0);
      
      //changing camMode can be used to switch between the normal targeting mode and light mode for driving
      table.getEntry("camMode").setNumber(0); //1 is light mode, 0 is normal
      
      table.getEntry("ledMode").setNumber(0);
      table.getEntry("stream").setNumber(0);

      xEntry = table.getEntry("tx");
      yEntry = table.getEntry("ty");
      targetFoundEntry = table.getEntry("tv");
    }
    catch (RuntimeException ex){
        DriverStation.reportError("error getting limelight table because: " + ex.getMessage(), true);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //maybe change default value to return the last x? Then you dont need lastX variable
    x = xEntry.getDouble(0); //ranges from -29.8 to 29.8 degrees for LL2
    y = yEntry.getDouble(0); //ranges from -24.85 to 24.85 degrees for LL2
    targetFound = targetFoundEntry.getDouble(0);
    //double area = table.getEntry("ta").getDouble(0); //ranges from 0 to 100% of image

    if (getTargetFound()) {
      lastX = x;
      lastY = y;
    }
    
    //posts to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightArea", area); 
    SmartDashboard.putNumber("LimelightTargetFound", targetFound);

    SmartDashboard.putNumber("distance simple", getTargetDistanceSimple()); 
    SmartDashboard.putNumber("distance actual", getTargetDistance()); 
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
    double distance = Constants.limelightToTargetHeight / Math.tan(Math.toRadians(lastY + Constants.limelightGoalMountDegreeOffset));
    
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
    /*distance = Constants.limelightToRobotCenterDistance + Constants.goalOutsideRadius + 
      Constants.targetDeltaY / (Math.tan(Math.toRadians(lastY + Constants.limelightMountDegreeOffset))
      * Math.cos(Math.toRadians(lastX)));

    SmartDashboard.putNumber("target distance", distance);*/
    //return distance;

    //law of cosines to still give the same distance when you rotate the turret
    //we care about the distance to the robot center not to the limelight
    double rawDistance = Constants.limelightToTargetHeight / (Math.tan(Math.toRadians(lastY + Constants.limelightGoalMountDegreeOffset))
      * Math.cos(Math.toRadians(lastX)));
    double radius = Constants.limelightToRobotCenterRadius;

    double distance = Math.sqrt(
      Math.pow(rawDistance, 2) + Math.pow(radius, 2) 
      - 2 * rawDistance * radius * Math.cos(Units.degreesToRadians(180 - lastX))
    );
    SmartDashboard.putNumber("distance turret corrected", distance);
    return distance;
  }
}
