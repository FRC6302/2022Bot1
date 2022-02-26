package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class VisionPoseEstimation {
    
  public VisionPoseEstimation() {

  }

  public static Pose2d getGlobalPoseEstimation(double distance, double gyroAngle, double turretAngle, double tx) {
    //normal math conventions
    /*return new Pose2d(
      distance * Math.sin(Units.degreesToRadians(gyroAngle + turretAngle - tx - 90)),
      -distance * Math.cos(Units.degreesToRadians(gyroAngle + turretAngle - tx - 90)), 
      Rotation2d.fromDegrees(gyroAngle));*/

    //wpilib conventions
    return new Pose2d(
      distance * Math.cos(Units.degreesToRadians(gyroAngle + turretAngle - tx - 90)), 
      distance * Math.sin(Units.degreesToRadians(gyroAngle + turretAngle - tx - 90)),
      Rotation2d.fromDegrees(gyroAngle));
  }

  public static double getCurrDistance(double distance, double gyroAngle, double turretAngle, double tx) {
    Pose2d pose = getGlobalPoseEstimation(distance, gyroAngle, turretAngle, tx);
    return Math.sqrt(Math.pow(pose.getX(), 2)  + Math.pow(pose.getY(), 2));
  }
  
}
