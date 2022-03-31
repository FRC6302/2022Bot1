// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeders;

public class FeedBoth extends CommandBase {
  Feeders feeders;
  double speed = 0;

  /** Creates a new FeedBoth. */
  public FeedBoth(Feeders feeders, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feeders = feeders;
    this.speed = speed;

    addRequirements(feeders);
  }

  /** Creates a new FeedBoth. */
  public FeedBoth(Feeders feeders) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feeders = feeders;

    addRequirements(feeders);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (speed == 0) {
      feeders.setBothMotors(0.5, 0.8);
    }
    else {
      feeders.setBothMotors(speed, speed);
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
