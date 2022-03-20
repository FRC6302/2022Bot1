// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.library.Utilities;
import frc.robot.subsystems.MecDriveTrain;

public class SixBall extends CommandBase {
  MecDriveTrain mecDriveTrain;
  PathPlannerTrajectory sixBallTrajectory = PathPlanner.loadPath("6 ball", 4, 1.5);

  /** Creates a new SixBall. */
  public SixBall(MecDriveTrain mecDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
    addRequirements(mecDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //USE SEQUENTIAL COMMAND INSTEAD OF THIS UGLINESS
    Utilities.getMecControllerCommand(sixBallTrajectory, mecDriveTrain).andThen(mecDriveTrain::stopDrive).schedule(true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
