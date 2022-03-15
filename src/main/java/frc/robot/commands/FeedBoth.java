// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederFront;
import frc.robot.subsystems.FeederMiddle;

public class FeedBoth extends CommandBase {
  FeederFront feederFront;
  FeederMiddle feederMiddle;

  /** Creates a new FeedBoth. */
  public FeedBoth(FeederFront feederFront, FeederMiddle feederMiddle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feederFront = feederFront;
    this.feederMiddle = feederMiddle;

    addRequirements(feederFront, feederMiddle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederFront.setMotor(0.4);
    feederMiddle.setMotor(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederFront.stopMotor();
    feederMiddle.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
