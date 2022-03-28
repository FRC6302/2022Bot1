// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.PPMecanumControllerCommand;
import frc.robot.subsystems.MecDriveTrain;

/** Add your docs here. */
public class Utilities {
  public static Command getMecControllerCommand(PathPlannerTrajectory ppTrajectory, MecDriveTrain mecDriveTrain) {
    // Create config for trajectory
    /*TrajectoryConfig config =
      new TrajectoryConfig(Constants.maxMecSpeed,Constants.maxMecAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(mecDriveTrain.getMecKinetimatics());
          
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);*/
    
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kpMecThetaController, 0., 0., 
      new Constraints(Constants.maxMecRotationVelocity, Constants.maxMecRotationAccel));
      
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.reset(mecDriveTrain.getPoseEstimate().getRotation().getRadians());

    //mecDriveTrain.setPose(ppTrajectory.getInitialPose());

    PPMecanumControllerCommand mecanumControllerCommand = new PPMecanumControllerCommand(
      ppTrajectory,
      mecDriveTrain::getPoseEstimate,
      //TODO go to WPILIB source code for mecControlCommand and see how they use this feedforward
      //and then use that in the mecDriveTrain.setSpeeds() method
      //mecDriveTrain.getMecFeedforward(),
      mecDriveTrain.getMecKinematics(),

      // Position contollers
      new PIDController(Constants.kpMecXController, 0, 0),
      new PIDController(Constants.kpMecYController, 0, 0),
      thetaController,

      // Needed for normalizing wheel speeds
      //Constants.maxMecSpeed,

      // Velocity PID's
      /*new PIDController(Constants.kpMecL1Velocity, 0, 0),
      new PIDController(Constants.kpMecL2Velocity, 0, 0),
      new PIDController(Constants.kpMecR1Velocity, 0, 0),
      new PIDController(Constants.kpMecR2Velocity, 0, 0),*/
      //mecDriveTrain::getCurrentWheelSpeeds,
      mecDriveTrain::setSpeeds, // Consumer for the output motor voltages
      mecDriveTrain
    );

    //PathPlannerState initialState = ppTrajectory.getInitialState();
    /*return new InstantCommand(() -> 
      mecDriveTrain.setPose(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation)))
      .andThen(mecanumControllerCommand);
      //.andThen(mecDriveTrain::stopDrive);*/

    return mecanumControllerCommand;
  } 
}
