// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.MecDriveTrain;

public class DriveMec extends CommandBase {
  private MecDriveTrain mecDriveTrain;

  /** Creates a new DriveMec. */
  public DriveMec(MecDriveTrain mecDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mecDriveTrain.setMotors(
      Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftStickX), 
      Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftStickY), 
      Robot.robotContainer.getDriverDeadzoneAxis(Constants.rightTrigger)
         - Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftTrigger), 
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
