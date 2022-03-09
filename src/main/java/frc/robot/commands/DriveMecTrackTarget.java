// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightGoal;
import frc.robot.subsystems.MecDriveTrain;

public class DriveMecTrackTarget extends CommandBase {
  private MecDriveTrain mecDriveTrain;

  double x = 0, y = 0, z = 0;

  /** Creates a new DriveMecTrackTarget. */
  public DriveMecTrackTarget(MecDriveTrain mecDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = -1 * Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftStickY);
    y = -1 * Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftStickX);
    z = -1/2 * mecDriveTrain.getPerpV(LimelightGoal.getLastX()) / LimelightGoal.getTargetDistance();

    mecDriveTrain.setMotors(x, y, z, true);
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
