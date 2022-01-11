// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Move extends CommandBase {
  DriveTrain driveTrain;
  double leftCommand;
  double rightCommand;
  double moveTime;
  Timer timer;
  private boolean finished = false;

  //default move command. runs if only the DriveTrain paramater is inputted to the command when it is called
  public Move(DriveTrain driveTrain) {
    /*this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    timer = new Timer();
    moveTime = Constants.MoveTime;  
    leftCommand = Constants.leftMotorsMoveSpeed;
    rightCommand = Constants.rightMotorsMoveSpeed;*/ 

    this(driveTrain, Constants.leftMotorsMoveSpeed, Constants.rightMotorsMoveSpeed, Constants.MoveTime);
  }  

  //runs when all 4 paramaters are inputted in to the command call
  public Move(DriveTrain driveTrain, double leftCommand, double rightCommand, double moveTime) {
    this.driveTrain = driveTrain;
    this.leftCommand = leftCommand;
    this.rightCommand = rightCommand;
    addRequirements(driveTrain);
    timer = new Timer();   
    this.moveTime = moveTime; 
  }  

  public Move(DriveTrain driveTrain, double distance, double speed) {
    /*pseudocode:
    get average of distance travelled
    left command = speed * curDistance / endDistance ?
    make a seperate move distance command
    */
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {   
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.reset();
    timer.start();
    while(timer.get() < moveTime)
    {
      driveTrain.setLeftMotors(leftCommand);
      driveTrain.setRightMotors(rightCommand);
    }
    finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}