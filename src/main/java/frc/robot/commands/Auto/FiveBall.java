// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.library.Utilities;
import frc.robot.subsystems.MecDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBall extends SequentialCommandGroup {
  MecDriveTrain mecDriveTrain;
  PathPlannerTrajectory sixBallTrajectory = PathPlanner.loadPath("6 ball", 4, 1.5);
  
  /** Creates a new FiveBall. */
  public FiveBall(MecDriveTrain mecDriveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.mecDriveTrain = mecDriveTrain;
    addRequirements(mecDriveTrain);
    
    addCommands(
      Utilities.getMecControllerCommand(sixBallTrajectory, mecDriveTrain),
      new InstantCommand(mecDriveTrain::stopDrive)
    );
  }
}
