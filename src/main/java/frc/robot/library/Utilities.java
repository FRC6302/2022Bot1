// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.PPMecanumControllerCommand;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.RobotState;

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
    thetaController.reset(RobotState.getPoseEstimate().getRotation().getRadians());

    //mecDriveTrain.setPose(ppTrajectory.getInitialPose());

    PPMecanumControllerCommand mecanumControllerCommand = new PPMecanumControllerCommand(
      ppTrajectory,
      RobotState::getPoseEstimate,
      //TODO go to WPILIB source code for mecControlCommand and see how they use this feedforward
      //and then use that in the mecDriveTrain.setSpeeds() method
      //mecDriveTrain.getMecFeedforward(),
      RobotState.getMecKinematics(),

      // Position contollers
      new PIDController(Constants.kpMecPosXController, 0, 0),
      new PIDController(Constants.kpMecPosYController, 0, 0),
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


  //returns the angle but between -180 and 180
  public static double constrainAngle(double rawAngle) {
    double angle = (rawAngle + 180.0) % 360.0;
    if (angle < 0) {
      angle += 360;
    }
    return angle - 180;
    
  }
}
