// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.FeedTimed;
import frc.robot.commands.SuckBalls;
import frc.robot.library.Utilities;
import frc.robot.subsystems.Feeders;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MecDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Still5Ball extends SequentialCommandGroup {
  MecDriveTrain mecDriveTrain;
  Intake intake;
  Feeders feeders;

  PathPlannerTrajectory path1 = PathPlanner.loadPath("Still5Ball1", 4, 2);
  PathPlannerTrajectory path2 = PathPlanner.loadPath("Still5Ball2", 4, 2);
  PathPlannerTrajectory path3 = PathPlanner.loadPath("Still5Ball3", 4, 2);
  PathPlannerTrajectory path4 = PathPlanner.loadPath("Still5Ball4", 4, 2);

  PathPlannerState initialState = path1.getInitialState();

  /** Creates a new Still5Ball. */
  public Still5Ball(MecDriveTrain mecDriveTrain, Intake intake, Feeders feeders) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.mecDriveTrain = mecDriveTrain;
    this.intake = intake;
    this.feeders = feeders;
    
    addCommands(
      new InstantCommand(() -> {
        mecDriveTrain.setPose(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
      }),
      parallel(
        Utilities.getMecControllerCommand(path1, mecDriveTrain),
        new SuckBalls(intake)
      ),
      new FeedTimed(feeders, 2),
      Utilities.getMecControllerCommand(path2, mecDriveTrain),
      Utilities.getMecControllerCommand(path3, mecDriveTrain),
      Utilities.getMecControllerCommand(path4, mecDriveTrain)
    );
  }
}
