// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightGoal;
import frc.robot.subsystems.Turret;

public class TurretFollowTarget extends CommandBase {
  Turret turret;
  double steeringAdjust;

  /** Creates a new TurretFollowTarget. */
  public TurretFollowTarget(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!LimelightGoal.getTargetFound()) {
      steeringAdjust = Constants.turretSeekSpeed;
    }
    else //this runs when the target is in view of camera
    {
      steeringAdjust = LimelightGoal.getX() / 100;
    }

    turret.setMotor(steeringAdjust);
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
