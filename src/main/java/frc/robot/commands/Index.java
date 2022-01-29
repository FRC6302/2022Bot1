// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Feeder;

public class Index extends CommandBase {
  double ballsPickedUp = 0, ballsShot = 0, ballsHolding = 0;

  /** Creates a new Index. */
  public Index() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballsPickedUp = ColorSensor.getBallsPickedUp();
    ballsShot = Feeder.getBallsShot();

    ballsHolding = ballsPickedUp - ballsShot;

    SmartDashboard.putNumber("balls holding", ballsHolding);
    
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
