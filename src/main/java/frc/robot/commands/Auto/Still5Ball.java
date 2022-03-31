// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Still5Ball extends ParallelCommandGroup {
  MecDriveTrain mecDriveTrain;
  Intake intake;
  Feeders feeders;
  Shooter shooter;
  Turret turret;
  Hood hood;

  PathPlannerTrajectory path1 = PathPlanner.loadPath("Still5Ball1", 0.5, 2);
  PathPlannerTrajectory path2 = PathPlanner.loadPath("Still5Ball2", 0.5, 2);
  PathPlannerTrajectory path3 = PathPlanner.loadPath("Still5Ball3", 0.5, 2);
  PathPlannerTrajectory path4 = PathPlanner.loadPath("Still5Ball4", 0.5, 2);

  PathPlannerState initialState = path1.getInitialState();

  /** Creates a new Still5Ball. */
  public Still5Ball(MecDriveTrain mecDriveTrain, Intake intake, Feeders feeders, Shooter shooter, Turret turret, Hood hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.mecDriveTrain = mecDriveTrain;
    this.intake = intake;
    this.feeders = feeders;
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;

    //addRequirements(mecDriveTrain, intake, feeders, shooter, turret, hood);

    //CHANGE TO PARALLEL SO THAT INTAKE IS RUNNNING THE WHOLE TIME
    
    //TODO dont have vision kick in for a few seconds
    addCommands(
      sequence(
        new InstantCommand(() -> {
            RobotState.setPose(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
            NavX.setAngleOffset(initialState.holonomicRotation.getDegrees());
        }),
        Utilities.getMecControllerCommand(path1, mecDriveTrain).andThen(mecDriveTrain::stopDrive),
        new WaitCommand(0.2), //gives time for shooter to get the right speed
        new FeedTimed(feeders, 2),

        Utilities.getMecControllerCommand(path2, mecDriveTrain).andThen(mecDriveTrain::stopDrive),
        new WaitCommand(0.2),
        new FeedTimed(feeders, 1.5),

        Utilities.getMecControllerCommand(path3, mecDriveTrain).andThen(mecDriveTrain::stopDrive),
        new WaitCommand(2),

        Utilities.getMecControllerCommand(path4, mecDriveTrain).andThen(mecDriveTrain::stopDrive),
        new WaitCommand(0.2),
        new FeedTimed(feeders, 1)
      ),
      new SuckBalls(intake),
      new TrackTargetCenterPose(false, mecDriveTrain, turret, hood, shooter)
    );
  }
}
