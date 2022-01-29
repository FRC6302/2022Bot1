// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  double ballsPickedUp = 0, ballsShot = 0, ballsHolding = 0;

  /** Creates a new Indexer. */
  public Indexer() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ballsPickedUp = ColorSensor.getBallsPickedUp();
    ballsShot = Feeder.getBallsShot();

    ballsHolding = ballsPickedUp - ballsShot;

    SmartDashboard.putNumber("balls holding", ballsHolding);
  }
}
