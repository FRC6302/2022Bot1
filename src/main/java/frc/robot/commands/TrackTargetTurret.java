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

public class TrackTargetTurret extends CommandBase {
  Turret turret;

  double distance = 3;

  Pose2d robotPose;
  double angleToTarget = 0;

  double vx = 0, vy = 0, angV = 0;

  double gyroYaw = 0;

  double x = 0,turretAngle = 0;

  double turretSetpoint = 0;

  boolean updateVisionOdom = true;
  
  /** Creates a new TrackTargetTurret. */
  public TrackTargetTurret(boolean updateVisionOdom, Turret turret) {
    this.turret = turret;
    this.updateVisionOdom = updateVisionOdom;

    // Use addRequirements() here to declare subsystem dependencies.
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
        RobotState.updateOdometryWithVision(distance, gyroYaw, turretAngle, x);
      }
      
      
    }
    robotPose = RobotState.getPoseEstimate();
    gyroYaw = robotPose.getRotation().getDegrees();
 
    distance = RobotState.getActualDistance();
 
    angleToTarget = Units.radiansToDegrees(Math.atan2(
      robotPose.getY() - Constants.goalLocation.getY(), 
      robotPose.getX() - Constants.goalLocation.getX())) - 180;

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
