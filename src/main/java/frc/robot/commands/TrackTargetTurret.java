// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightGoal;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Turret;

//command for the turret to always track the goal as we drive around the field
public class TrackTargetTurret extends CommandBase {
  Turret turret;

  double distance = 3;

  Pose2d robotPose;
  double angleToTarget = 0;

  double vx = 0, vy = 0, angV = 0;

  double gyroYaw = 0;

  double x = 0, turretAngle = 0;


  //sometimes you want to ignore what the limelight is saying if it's giving bad data. If this bool is false, we won't use vision to update the odometry
  boolean updateVisionOdom = true;

  /** Creates a new TrackTargetTurret. */
  public TrackTargetTurret(boolean updateVisionOdom, Turret turret) {
    this.turret = turret;
    this.updateVisionOdom = updateVisionOdom;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose = RobotState.getPoseEstimate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroYaw = NavX.getGyroYaw();
    turretAngle = turret.getRobotRelAngle();


    if (LimelightGoal.getTargetFound()) { //runs when the LL can see the target
      distance = LimelightGoal.getTargetDistance();
      x = LimelightGoal.getX();
      
      if (updateVisionOdom) {
        //it would make more sense to do this in RobotState and have a static method for turning vision odom off and on
        RobotState.updateOdometryWithVision(distance, gyroYaw, turretAngle, x);
      }
      
      
    }
    robotPose = RobotState.getPoseEstimate();
    gyroYaw = robotPose.getRotation().getDegrees();
 
    distance = RobotState.getActualDistance();
 
    //not leading shot
    angleToTarget = Units.radiansToDegrees(Math.atan2(
      robotPose.getY() - Constants.goalLocation.getY(), 
      robotPose.getX() - Constants.goalLocation.getX())) - 180;

    //leading shot
    /*angleToTarget = Units.radiansToDegrees(Math.atan2(
      robotPose.getY() - RobotState.getEffectiveGoalPose().getY(), 
      robotPose.getX() - RobotState.getEffectiveGoalPose().getX())) - 180;*/

    vx = RobotState.getGlobalMecVx();
    vy = RobotState.getGlobalMecVy();
    angV = NavX.getGyroAngV();

    turret.setMotorPosPID(robotPose, angleToTarget - gyroYaw, vx, vy, angV, distance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
