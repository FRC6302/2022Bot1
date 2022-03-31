// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.kauailabs.navx.AHRSProtocol.MagCalData;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FeedTimed;
import frc.robot.commands.SuckBalls;
import frc.robot.commands.TrackTargetCenterPose;
import frc.robot.library.Utilities;
import frc.robot.subsystems.Feeders;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Still2BallRight extends ParallelCommandGroup {
  PathPlannerTrajectory path = PathPlanner.loadPath("Still2BallRight1", 1, 1);

  MecDriveTrain mecDriveTrain;
  Intake intake;
  Feeders feeders;
  Shooter shooter;
  Turret turret;
  Hood hood;

  PathPlannerState initialState = path.getInitialState();

  /** Creates a new Still2Ball. */
  public Still2BallRight(MecDriveTrain mecDriveTrain, Intake intake, Feeders feeders, Shooter shooter, Turret turret, Hood hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
    this.intake = intake;
    this.feeders = feeders;
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;

    //addRequirements(mecDriveTrain, intake, feeders, shooter, turret, hood);

    addCommands(
      sequence(
        new InstantCommand(() -> {
          NavX.zeroGyroYaw();
          NavX.setAngleOffset(initialState.holonomicRotation.getDegrees());
          RobotState.setPose(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
          //SmartDashboard.putNumber("HERE", 1);
          
        }),
        new WaitCommand(0.1),
        Utilities.getMecControllerCommand(path, mecDriveTrain).andThen(mecDriveTrain::stopDrive),
        new WaitCommand(0.2), //gives time for shooter to get the right speed
        new FeedTimed(feeders, 2)

      ),
      new SuckBalls(intake),
      new TrackTargetCenterPose(true, mecDriveTrain, turret, hood, shooter)
    );
  }


}
