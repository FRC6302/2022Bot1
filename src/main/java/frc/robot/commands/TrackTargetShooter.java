// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Shooter;

public class TrackTargetShooter extends CommandBase {
  Shooter shooter;

  double distance = 3;

  /** Creates a new TrackTargetShooter. */
  public TrackTargetShooter(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = RobotState.getActualDistance();

    //for shooting while moving
    //distance = RobotState.getEffectiveDistance();

    shooter.setMotorsVelPID(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
