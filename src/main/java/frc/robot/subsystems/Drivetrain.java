// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
  NOTES FOR NEW CODERS: 
  You should probably do the AP CSA CodeHS course before trying to code 
  for robotics. It was the most helpful course i've done, and the beginning is fun with Karel if you
  want to do it. Make sure you completely understand all the stuff about classes, objects, and methods.
  You can probably skip some of the string stuff, but it's still good to know.
  Once you have understand everything in the course, watch these series to apply your knowledge to FRC:
  1. FRC Java Programming Tutorials by Manning Robotics
  https://www.youtube.com/playlist?list=PLqolGlJdb9oWsgP4biujl_eTFazESWl8o 
  Note that this series is a little old, and there has been a new command based framework since then. 
  So don't copy his code at all. Just follow along with the logic of it.
  Note that RobotContainer is the new version of OI, and Constants is the new RobotMap.
  2. First Robotics Competition - Command Base System 2020 - VS Code - Java
  https://www.youtube.com/playlist?list=PLYwJIUT_B-n612Gqmfsq1ukYLa6WKgonc 
  This video uses the new framework.
  I would recommend that you follow along these videos without looking at my code until you have everything
  working, especially if you encounter some errors. It's a learning experience to solve your own errors.
  Good luck :) -Samuel, 2021
  
  
  FRC coding tips:
  1. Refrain from using wait() functions or while loops anywhere if possible cuz it can mess with the how the
  FRC scheduler and other stuff work and update. It might cause weird errors.
  2. If you have an error, someone has probably posted about it on Chief Delphi or Stack Overflow. 
  Google the error and see what pops up. It could also be a mechanical problem, especially if Pauley built it. 
  3. WPILIB has a lot of tutorials for various things on their website.
  Various guides:
  1. PID Control explanation
  Channel: FRC 0 to Autonomous
  https://www.youtube.com/channel/UCmJAoN-yI6AJDv7JJ3372yg
  Watch parts 3 and 4 for the logic of PID control. I wouldn't copy the code directly, as it doesn't 
  use the command-based framework like we do. 
  2. FRC Programming Done Right
  https://www.chiefdelphi.com/t/frc-programming-done-right/158005
  A guide for basically everything you'll code for FRC
  
  Putting your code on GitHub: 
  1. Watch a few videos on how GitHub works - learn about pull requests, commits, pushes, branches, etc.
  2. Watch the GitHub VS Code tutorial video here:
  FRC - Command Base System - Java - VS Code 2020 - Part 10 by Nevin Morrison
  https://www.youtube.com/watch?v=goUt-VxCnE0&t=1s 
  3. It is good practice to commit after every day of work. Make sure code builds/deploys before commiting
  4. Write good commit messages
*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//test commit
//test from kaden

public class DriveTrain extends SubsystemBase {
  private static DriveTrain driveTrain;

  WPI_TalonSRX motorL1 = new WPI_TalonSRX(Constants.motorL1Value);
  //WPI_TalonSRX motorL2 = new WPI_TalonSRX(Constants.motorL2Value);
  WPI_TalonSRX motorR1 = new WPI_TalonSRX(Constants.motorR1Value);
  //WPI_TalonSRX motorR2 = new WPI_TalonSRX(Constants.motorR2Value);

  //brake mode reduces wheel slip
  private final NeutralMode motorMode = NeutralMode.Brake;

  /** Creates a new Drivetrain. */
  private DriveTrain() {
    motorL1.setNeutralMode(motorMode);
    //motorL2.setNeutralMode(motorMode);
    motorR1.setNeutralMode(motorMode);
    //motorR2.setNeutralMode(motorMode);

    motorL1.setSafetyEnabled(true);
    //motorL2.setSafetyEnabled(true);
    motorR1.setSafetyEnabled(true);
    //motorR2.setSafetyEnabled(true);

    //motorL1.setExpiration(30);
    //motorL2.setExpiration(30);
    //motorR1.setExpiration(10);
    //motorR2.setExpiration(10);

    //this is the deadzone for the motors. Any number below this get changed to zero.
    //Note that the minimum motor input to overcome static friction is about 0.0015 ish
    motorL1.configNeutralDeadband(0.001);
    //motorL2.configNeutralDeadband(0.001);
    motorR1.configNeutralDeadband(0.001);
    //motorR2.configNeutralDeadband(0.001);
    
    //TODO: play with this
    //motorL1.configPeakOutputForward(0.9);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //this is supposed to be so i dont get errors saying the motor output doesnt update enough,
    //but i still get them
    motorL1.feed();
    //motorL2.feed();
    motorR1.feed();
    //motorR2.feed();
  }

  public static synchronized DriveTrain getInstance(){
    if (driveTrain == null){
      driveTrain = new DriveTrain();
    }

    return driveTrain;
  }

  public synchronized void setLeftMotors(double speed){
    motorL1.set(ControlMode.PercentOutput, speed);
    //motorL2.set(ControlMode.PercentOutput, speed);
  }

  //right motors have inverted speed bc of how the motors are oriented on robot
  public synchronized void setRightMotors(double speed){
    motorR1.set(ControlMode.PercentOutput, -speed);
    //motorR2.set(ControlMode.PercentOutput, -speed);
  }

  public synchronized void stopDrive() {
    motorL1.set(ControlMode.PercentOutput, 0);
    //motorL2.set(ControlMode.PercentOutput, 0);
    motorR1.set(ControlMode.PercentOutput, 0);
    //motorR2.set(ControlMode.PercentOutput, 0);
  }
}
