// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

//unused
public class ResetTurret extends CommandBase {
  Turret turret;

  double initialAngle = 0;

  //determines which way to turn the turret
  boolean angleTooLow = false;

  /** Creates a new ResetTurret. */
  public ResetTurret(Turret turret, double initialAngle, boolean angleTooLow) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.initialAngle = initialAngle;
    this.angleTooLow = angleTooLow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //initialAngle = turret.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (angleTooLow) { 
      turret.turnToAngle(initialAngle + 360);
    }
    else {
      turret.turnToAngle(initialAngle - 360);
    }
    
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
