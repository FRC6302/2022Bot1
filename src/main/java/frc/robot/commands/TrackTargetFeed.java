// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeders;
import frc.robot.subsystems.Turret;

//controls the feeder
public class TrackTargetFeed extends CommandBase {
  Feeders feeders;
  Turret turret;

  /** Creates a new TrackTargetFeed. */
  public TrackTargetFeed(Feeders feeders, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feeders = feeders;
    this.turret = turret;

    addRequirements(feeders);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //don't feed balls while the turret is resetting, or a ball will be shot in some random direction
    if (turret.getIsResetting()) {
      feeders.stopBothMotors();
    }
    else {
      feeders.setBothMotors(.5, .8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeders.stopBothMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
