// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeders;

public class FeedTimed extends CommandBase {
  Feeders feeders;
  double time;

  Timer timer = new Timer();
  /** Creates a new FeedTimed. */
  public FeedTimed(Feeders feeders, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feeders = feeders;
    this.time = time;

    addRequirements(feeders);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeders.setBothMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
