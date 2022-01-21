// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  WPI_TalonSRX motorShooter = new WPI_TalonSRX(Constants.motorShooter);

  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //hi

  //new feature 3

  public void shootWithInitialBallVelocity(double paraV, double perpV, double hoodAngle, double offsetAngle, double distance) {
    //offsetAngle = 10 * perpV;
    
    double shotInitV = Math.pow(Math.pow(perpV / Math.tan(offsetAngle) + paraV, 2) 
      + Math.pow(perpV, 2) - 9.807 * Math.pow(distance, 2) * Math.pow(Math.sin(hoodAngle), 2) 
      / (2 * Constants.targetDeltaY * Math.pow(Math.cos(hoodAngle), 2) - 2 * distance 
      * Math.cos(hoodAngle) * Math.sin(hoodAngle)), 0.5);

    setMotor(shotInitV / 10);
  }

  public void setMotor(double speed) {
    motorShooter.set(ControlMode.PercentOutput, speed);
  }

  
}
