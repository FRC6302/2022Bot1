// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurnTurret extends CommandBase {
  Turret turret;
  /** Creates a new TurnTurret. */
  public TurnTurret(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //turret.setMotor(Robot.robotContainer.getDriverRawAxis(Constants.rightStickX) / -4);
    turret.setMotorPosPID(new Pose2d(), 0, 0, 0, 0, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
