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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  WPI_TalonSRX motorL1 = new WPI_TalonSRX(Constants.motorL1Value);
  WPI_TalonSRX motorL2 = new WPI_TalonSRX(Constants.motorL2Value);
  WPI_TalonSRX motorR1 = new WPI_TalonSRX(Constants.motorR1Value);
  WPI_TalonSRX motorR2 = new WPI_TalonSRX(Constants.motorR2Value);

  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
