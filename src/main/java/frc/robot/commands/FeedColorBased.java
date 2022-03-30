// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Feeders;


public class FeedColorBased extends CommandBase {
  //FeederFront feederFront;
  //FeederMiddle feederMiddle;
  Feeders feeders;

  //keeps track of how long an opposing ball has been sitting in front of the color sensor
  //Timer timer;
  
  /** Creates a new FeedColorBased. */
  public FeedColorBased(Feeders feeders) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feeders = feeders;

    addRequirements(feeders);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if the ball is our alliance then send it to the shooter to score
    if (ColorSensor.getCurrentBallIsAlliance()) {
      //timer.reset();
      feeders.setBothMotorVolts(Constants.feederFrontDefaultVolts, Constants.feederMiddleDefaultVolts);
    }
    else { 
      //timer.start(); //doesnt effect the timer if its already running

      //if the ball is the wrong color then we have to give everything time to get into position to miss
      if (ColorSensor.getTimeSeeingOppositeBall() > Constants.waitToFeedOppositeBallTime) {
        feeders.setBothMotorVolts(Constants.feederFrontDefaultVolts, Constants.feederMiddleDefaultVolts);
      }
      else {
        feeders.stopBothMotors();
      }
    }



    //if we have seen a red ball in the last 3 seconds - doesnt work for both mounting spots because it stops after you send the ball to shoot because it has seen red recently
    /*if (ColorSensor.getTimeSinceOppositeBall() + ColorSensor.getTimeSeeingOppositeBall() < Constants.waitToFeedOppositeBallTime) {
      feeders.stopBothMotors();
    }
    else {
      feeders.setBothMotorVolts(Constants.frontFeederDefaultVolts, Constants.middleFeederDefaultVolts);
    }*/
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
