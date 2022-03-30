// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.library.Utilities;
import frc.robot.subsystems.LimelightBall;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.RobotState;

public class TrackBall extends CommandBase {
  MecDriveTrain mecDriveTrain;
  Pose2d latestBallPose = new Pose2d();
  Pose2d currPose = new Pose2d();

  // Create config for trajectory
  TrajectoryConfig config = new TrajectoryConfig(Constants.maxMecSpeed, Constants.maxMecAcceleration)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(RobotState.getMecKinematics());

  //ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();

  PathPlannerTrajectory latestTrajectory;

  CommandBase mecCommand = new InstantCommand();

  private int loopCount = 0;
  private int loopsBetweenGenerations = (int) Math.round(Constants.timeBetweenTrajectoryGeneration / Constants.loopTime);

  /** Creates a new TrackBall. */
  public TrackBall(MecDriveTrain mecDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;

    //if I added drivetrain as a requirement here then doing the other command below would stop this one?
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopCount = 0;

    currPose = RobotState.getPoseEstimate();
    latestBallPose = LimelightBall.getNearestBallPose(currPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //recalculates the trajectory and command every so often to account for the ball moving. Doing it constantly would be inefficient
    if (loopCount % loopsBetweenGenerations == 0) {
      currPose = RobotState.getPoseEstimate();
      latestBallPose = LimelightBall.getNearestBallPose(currPose);

      latestTrajectory = (PathPlannerTrajectory) TrajectoryGenerator.generateTrajectory(List.of(currPose, latestBallPose), config);

      //Find better way?
      mecCommand = (CommandBase) Utilities.getMecControllerCommand(latestTrajectory, mecDriveTrain);
      mecCommand.addRequirements(mecDriveTrain);
      mecCommand.initialize();
    }
    
    mecCommand.execute();
    //command.schedule(true);

    loopCount++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceError = currPose.getTranslation().getDistance(latestBallPose.getTranslation());
    double rotationError = Math.abs(currPose.getRotation().getDegrees() - latestBallPose.getRotation().getDegrees());

    return (distanceError < Constants.ballTrackingDistanceTolerance) && (rotationError < Constants.ballTrackingDegTolerance);
  }
}
