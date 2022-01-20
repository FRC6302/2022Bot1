// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  WPI_TalonSRX motorHood = new WPI_TalonSRX(Constants.motorHood);

  /** Creates a new Hood. */
  public Hood() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHoodAngle(double angle) {

  }

  public void setMotor(double speed) {
    motorHood.set(ControlMode.PercentOutput, speed);
  }
}
