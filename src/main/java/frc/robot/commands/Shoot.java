// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  Shooter shooter;
  double topSpeed = 0, bottomSpeed = 0;
  
  /**
   * Creates a new Shoot.
   */
  public Shoot(Shooter shooter, double topSpeed, double bottomSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.topSpeed = topSpeed;
    this.bottomSpeed = bottomSpeed;

    addRequirements(shooter); 
  }

  public Shoot(Shooter shooter) {
    this.shooter = shooter;
    topSpeed = 0.1;
    bottomSpeed = 0.1;

    addRequirements(shooter); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooter.setMotors(topSpeed, bottomSpeed);
    //shooter.setTopMotor(0.2);
    //shooter.setMotorsStateSpace(1);
    shooter.setMotorsVelPID(1);
    
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
