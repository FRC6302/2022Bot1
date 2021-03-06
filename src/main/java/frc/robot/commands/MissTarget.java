// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightGoal;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;


public class MissTarget extends CommandBase {
  /** Creates a new MissTarget. */
  MecDriveTrain mecDriveTrain;
  Turret turret;
  Hood hood;
  Shooter shooter;

  double distance = 3;

  Pose2d robotPose;
  double angleToTarget = 0;

  double vx = 0, vy = 0, angV = 0;

  double gyroYaw = 0;

  double turretSetpoint = 0;


  
  /** Creates a new TrackTarget. */
  public MissTarget(MecDriveTrain mecDriveTrain, Turret turret, Hood hood, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;

    addRequirements(shooter, turret, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose = RobotState.getPoseEstimate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = RobotState.getPoseEstimate();
    gyroYaw = robotPose.getRotation().getDegrees();
 
    distance = RobotState.getActualDistance();
 

    angleToTarget = Units.radiansToDegrees(Math.atan2(
      robotPose.getY() - Constants.goalLocation.getY(), 
      robotPose.getX() - Constants.goalLocation.getX())) - 180;

    vx = RobotState.getGlobalMecVx();
    vy = RobotState.getGlobalMecVy();
    angV = NavX.getGyroAngV();

    turretSetpoint = angleToTarget - gyroYaw;

    if (turretSetpoint + Constants.turretOffsetForMissing + 20 < Constants.maxTurretAngle) {
      turret.setMotorPosPID(robotPose, turretSetpoint + Constants.turretOffsetForMissing, vx, vy, angV, distance);    
    }
    else { //if it cant offset to the left, then do it to the right instead
      turret.setMotorPosPID(robotPose, turretSetpoint - Constants.turretOffsetForMissing, vx, vy, angV, distance);     
    }

    shooter.missTarget();
    hood.missTarget();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
    shooter.stopMotors();
    hood.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
