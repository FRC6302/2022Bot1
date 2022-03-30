package frc.robot.library;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class VisionPoseEstimation {
    
  public VisionPoseEstimation() {
  }

  /*gets the current pose (position) of the robot (relative to goal) based on limelight, turret, and gyro data.
    Uses polar coordinates and treats the goal and the origin, with distance as the magnitude and theta as
    the angle between the field x-axis and the goal
  */
  //TODO: add argument for current pose and check to see that the vision pose isnt too far off the drive pose because of bad limelight data
  public static Pose2d getGlobalPoseEstimation(Pose2d oldPose, double distance, double gyroAngle, double turretAngle, double tx) {
    //normal math conventions
    /*return new Pose2d(
      distance * Math.sin(Units.degreesToRadians(gyroAngle + turretAngle - tx - 90)),
      -distance * Math.cos(Units.degreesToRadians(gyroAngle + turretAngle - tx - 90)), 
      Rotation2d.fromDegrees(gyroAngle));*/

    //wpilib conventions
    //TODO: you are using the old tx with the new gyro and turret angle, not as accurate as it could be?
    Pose2d poseEstimate = new Pose2d(
      -distance * Math.cos(Units.degreesToRadians(gyroAngle + turretAngle - tx)), 
      -distance * Math.sin(Units.degreesToRadians(gyroAngle + turretAngle - tx)),
      Rotation2d.fromDegrees(gyroAngle));
    //).transformBy(new Transform2d(new Translation2d(Constants.goalLocation.getX(), Constants.goalLocation.getY()), new Rotation2d()));

    Pose2d transformedPose = new Pose2d(poseEstimate.getX() + Constants.goalLocation.getX(), 
      poseEstimate.getY() + Constants.goalLocation.getY(), 
      poseEstimate.getRotation());
    
    //double poseDelta = poseEstimate.getTranslation().getDistance(oldPose.getTranslation());
    //failsafe in case the calculation is messed up due to bad sensors or whatever
    /*if (poseDelta > Constants.visionPoseDeltaTolerance) {
      DriverStation.reportWarning("VISION POSE ESTIMATION IS REALLY OFF, CHECK INDIVIDUAL SENSOR DATA", false);
      return oldPose;
    }*/

    return transformedPose;
  }

/*public static double getCurrDistance(double distance, double gyroAngle, double turretAngle, double tx) {
    Pose2d pose = getGlobalPoseEstimation(distance, gyroAngle, turretAngle, tx);
    return Math.sqrt(Math.pow(pose.getX(), 2)  + Math.pow(pose.getY(), 2));
  }*/
  
}
