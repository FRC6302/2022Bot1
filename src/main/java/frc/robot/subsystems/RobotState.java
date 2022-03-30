// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.library.Data;
import frc.robot.library.MecanumDrivePoseEstimator;
import frc.robot.library.VisionPoseEstimation;

public class RobotState extends SubsystemBase {
  private static MecDriveTrain mecDriveTrain;

  //robot width wheel-to-wheel is 0.584 m, length wheel-to-wheel is 0.521 m
  //0.584/2 = 0.292, 0.521/2 = 0.2605
  //use (y, -x) if using normal math graphs, the origin is center of robot
  private static final Translation2d frontLeftLocation = new Translation2d(Constants.robotWheelToWheelLength/2, Constants.robotWheelToWheelWidth/2);
  private static final Translation2d frontRightLocation = new Translation2d(Constants.robotWheelToWheelLength/2, -Constants.robotWheelToWheelWidth/2);
  private static final Translation2d backLeftLocation = new Translation2d(-Constants.robotWheelToWheelLength/2, Constants.robotWheelToWheelWidth/2);
  private static final Translation2d backRightLocation = new Translation2d(-Constants.robotWheelToWheelLength/2, -Constants.robotWheelToWheelWidth/2);

  private static final MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
  
  

  private static Field2d field = new Field2d();

  private static ChassisSpeeds curChassisSpeeds = new ChassisSpeeds();

  private static Pose2d robotPose = new Pose2d(Constants.goalLocation, NavX.getGyroRotation2d());
  private static double actualDistance = 3;
  private static double airTime = Data.getAirtime(actualDistance);
  
  private static final MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(
    NavX.getGyroRotation2d(),
    robotPose,
    kinematics,
    VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(3)),
    VecBuilder.fill(Units.degreesToRadians(5)), //wheels slip a lot so their readings are less accurate
    VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1))
  );

  private static MedianFilter vxFilter = new MedianFilter(Constants.vxFilterSize);
  private static MedianFilter vyFilter = new MedianFilter(Constants.vyFilterSize);

  
  
  /** Creates a new RobotState. */
  public RobotState(MecDriveTrain mecDriveTrain) {
    RobotState.mecDriveTrain = mecDriveTrain;

    //how accurate the vision pose estimation is. The error comes from gyro and turret encoder drift and limelight targeting the wrong thing
    //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.001));

    //use this to show balls on the field?
    //field.getObject("red balls").setPose(new Pose2d());
    

    SmartDashboard.putData("field", field);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //updates the drive odometry every loop. Not updating using vision here because we prob wont be able to see the goal the whole match
    updateOdometry();

    //used for converting robot velocity into target-relative velocity
    curChassisSpeeds = kinematics.toChassisSpeeds(mecDriveTrain.getCurrentWheelSpeeds());
    
    robotPose = getPoseEstimate();
    actualDistance = robotPose.getTranslation().getDistance(Constants.goalLocation);
    airTime = Data.getAirtime(actualDistance);

    SmartDashboard.putNumber("pose x", robotPose.getX());
    SmartDashboard.putNumber("pose y", robotPose.getY());
    
    field.setRobotPose(robotPose);

    SmartDashboard.putNumber("mecanum vx", getGlobalMecVx());
    SmartDashboard.putNumber("mecanum vy", getGlobalMecVy());
    //SmartDashboard.putNumber("mecanum ang v", getAngV());

    SmartDashboard.putNumber("effective distance", getEffectiveDistance());
    SmartDashboard.putNumber("offset angle deg", getOffsetAngleDeg(getEffectiveDistance()));
  }

  /*
    Effective distance is the distance that the ball should be fired at to shoot while moving.
    This is different than the actual physical distance to the goal. It's the distance from the robot to the where 
    the goal will be after airTime seconds.The effective distance is used to calculate shooter
    speed, hood angle, and turret feedforward. 
  */
  public static double getEffectiveDistance() { 
    double vx = getGlobalMecVx();
    double vy = getGlobalMecVy();
    return Math.sqrt(actualDistance * actualDistance + airTime * airTime * (vx * vx + vy * vy) 
    - 2 * airTime * Math.sqrt(vx * vx + vy * vy) * Math.cos(Math.PI/2 + Math.atan2(-vy, -vx))
    );

    /*Pose2d goalRel = getGoalRelPose();

    return goalRel.getTranslation().getDistance(
      new Translation2d(
        //Constants.goalLocation.getX() - getVx() * airTime, 
        //Constants.goalLocation.getY()/* - getVy() * airTime
        0 - getGlobalMecVx() * airTime,
        0 - getGlobalMecVy() * airTime
      )
    );*/
  }

  public static double getOffsetAngleDeg(double effectiveDistance) {
    //Pose2d goalRel = getGoalRelPose();
    Pose2d pose = new Pose2d(robotPose.getX() - Constants.goalLocation.getX(), robotPose.getY() - Constants.goalLocation.getY(), robotPose.getRotation());
    return Units.radiansToDegrees( 
      Math.asin(airTime * 
        (getGlobalMecVy() * pose.getX() + getGlobalMecVx() * pose.getY())
        / (LimelightGoal.getTargetDistance() * effectiveDistance)
      )
    );
  }

  public static void updateOdometry() {
    robotPose = poseEstimator.update(NavX.getGyroRotation2d(), mecDriveTrain.getCurrentWheelSpeeds());
  }

   /** Updates the field relative position of the robot. */
  public  static void updateOdometryWithVision(double distance, double gyroAngle, double turretAngle, double tx) {
    //odometry.update(NavX.getGyroRotation2d(), getCurrentWheelSpeeds());
    updateOdometry();

    poseEstimator.addVisionMeasurement(
      VisionPoseEstimation.getGlobalPoseEstimation(getPoseEstimate(), distance, gyroAngle, turretAngle, tx), 
      Timer.getFPGATimestamp() - Constants.limelightLatency);
  }

  public static void setPose(Pose2d pose) {
    mecDriveTrain.resetEncoders();
    poseEstimator.resetPosition(pose, NavX.getGyroRotation2d());
  }

  public  static MecanumDriveKinematics getMecKinematics() {
    return kinematics;
  }

  public static Pose2d getPoseEstimate() {
    return poseEstimator.getEstimatedPosition();
  }

  //gets the pose of the robot relative to the goal. Makes the math simpler/cleaner
  public static Pose2d getGoalRelPose() {
    //the robot pose relative to the origin of the field (bottom right)
    Pose2d originRel = getPoseEstimate();

    return new Pose2d(originRel.getX() - Constants.goalLocation.getX(), 
    originRel.getY() - Constants.goalLocation.getY(), 
    originRel.getRotation());
  }

  public  static double getParaV(double turretAngleDeg) {
    //converts from robot-relative velocities directly to target relative velocities
    //double vz = curChassisSpeeds.vxMetersPerSecond; //velocity in the foward direction, relative to robot
    //double vx = -curChassisSpeeds.vyMetersPerSecond; //velocity towards the right, relative to robot
    double theta = Math.toRadians(90 - turretAngleDeg);
    //double output = vx * Math.cos(theta) + vz * Math.sin(theta);

    double output = -curChassisSpeeds.vyMetersPerSecond * Math.cos(theta) + curChassisSpeeds.vxMetersPerSecond * Math.sin(theta);

    SmartDashboard.putNumber("paraV", output);
    return output;
  }

  public static double getPerpV(double turretAngleDeg) {
    //converts from robot-relative velocities directly to target relative velocities
    //double vz = curChassisSpeeds.vxMetersPerSecond; //velocity in the foward direction, relative to robot
    //double vx = -curChassisSpeeds.vyMetersPerSecond; //velocity towards the right, relative to robot
    double theta = Math.toRadians(90 - turretAngleDeg);

    //double output = -vx * Math.sin(theta) + vz * Math.cos(theta);
    double output = curChassisSpeeds.vyMetersPerSecond * Math.sin(theta) + curChassisSpeeds.vxMetersPerSecond * Math.cos(theta);

    SmartDashboard.putNumber("perpV", output);
    return output;
  }

  public static double getAngV() {
    double output = Math.toDegrees(curChassisSpeeds.omegaRadiansPerSecond);
    //SmartDashboard.putNumber("angv", output);
    return output;
  }

  public static double getRobotRelVx() {
    //return -curChassisSpeeds.vyMetersPerSecond;
    /*double vx = curChassisSpeeds.vxMetersPerSecond; 
    SmartDashboard.putNumber("unfiltered mec vx", vx);
    SmartDashboard.putNumber("filtered mec vx", vxFilter.calculate(vx));
    //SmartDashboard.putNumber("gy", value)
    NetworkTableInstance.getDefault().flush();*/

    return vxFilter.calculate(curChassisSpeeds.vxMetersPerSecond);
  }

  public static double getRobotRelVy() {
    //return curChassisSpeeds.vxMetersPerSecond;
    return vyFilter.calculate(curChassisSpeeds.vyMetersPerSecond);
  }

  public static double getGlobalMecVx() {
    //double gyroAngle = NavX.getGyroYaw();
    //double gyroVx = NavX.getGyroGlobalVx();

    //Translation2d robotRelativeV = new Translation2d(getVx(), getVy());
    //plug into unscented kalman here
    return new Translation2d(getRobotRelVx(), getRobotRelVy()).rotateBy(NavX.getGyroRotation2d()).getX();
  }

  public static double getGlobalMecVy() {
    return new Translation2d(getRobotRelVx(), getRobotRelVy()).rotateBy(NavX.getGyroRotation2d()).getY();
  }
}
