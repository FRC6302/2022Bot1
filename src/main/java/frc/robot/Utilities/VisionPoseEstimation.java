package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class VisionPoseEstimation {
    
  public VisionPoseEstimation() {

  }

  /*gets the current pose (position) of the robot (relative to goal) based on limelight, turret, and gyro data.
    Uses polar coordinates and treats the goal and the origin, with distance as the magnitude and theta as
    the angle between the field x-axis and the goal
  */
  //TODO: add argument for current pose and check to see that it isn't too different
  public static Pose2d getGlobalPoseEstimation(double distance, double gyroAngle, double turretAngle, double tx) {
    //normal math conventions
    /*return new Pose2d(
      distance * Math.sin(Units.degreesToRadians(gyroAngle + turretAngle - tx - 90)),
      -distance * Math.cos(Units.degreesToRadians(gyroAngle + turretAngle - tx - 90)), 
      Rotation2d.fromDegrees(gyroAngle));*/

    //wpilib conventions
    return new Pose2d(
      -distance * Math.cos(Units.degreesToRadians(gyroAngle + turretAngle - tx)), 
      -distance * Math.sin(Units.degreesToRadians(gyroAngle + turretAngle - tx)),
      Rotation2d.fromDegrees(gyroAngle));
  }

  public static double getCurrDistance(double distance, double gyroAngle, double turretAngle, double tx) {
    Pose2d pose = getGlobalPoseEstimation(distance, gyroAngle, turretAngle, tx);
    return Math.sqrt(Math.pow(pose.getX(), 2)  + Math.pow(pose.getY(), 2));
  }
  
}
