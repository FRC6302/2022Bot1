// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Hood;

public class RaiseHood extends CommandBase {
  Hood hood;

  /** Creates a new RaiseHood. */
  public RaiseHood(Hood hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hood = hood;

    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double output = -Robot.robotContainer.getDriverDeadzoneAxis(Constants.rightStickY) / 4;
    //SmartDashboard.putNumber("hood motor speed command", output);
    //hood.setMotor(output);

    hood.setMotorPosPID(9, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
