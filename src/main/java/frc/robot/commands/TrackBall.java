// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.library.Utilities;
import frc.robot.subsystems.LimelightBall;
import frc.robot.subsystems.MecDriveTrain;

public class TrackBall extends CommandBase {
  MecDriveTrain mecDriveTrain;
  Pose2d latestBallPose = new Pose2d();
  Pose2d currPose = new Pose2d();

  // Create config for trajectory
  TrajectoryConfig config = new TrajectoryConfig(Constants.maxMecSpeed, Constants.maxMecAcceleration)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(mecDriveTrain.getMecKinematics());

  ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();

  PathPlannerTrajectory latestTrajectory;
  /** Creates a new TrackBall. */
  public TrackBall(MecDriveTrain mecDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currPose = mecDriveTrain.getPoseEstimate();
    latestBallPose = LimelightBall.getNearestBallPose(currPose);

    waypoints.add(currPose);
    waypoints.add(latestBallPose);
    
    latestTrajectory = (PathPlannerTrajectory) TrajectoryGenerator.generateTrajectory(waypoints, config);

    //interrupts the previous command with the new trajectory. Find better way?
    Utilities.getMecControllerCommand(latestTrajectory, mecDriveTrain).schedule(true);

    waypoints.clear();
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
