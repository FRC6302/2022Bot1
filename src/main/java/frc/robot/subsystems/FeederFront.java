// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederFront extends SubsystemBase {
  private WPI_TalonSRX motorFeederFront = new WPI_TalonSRX(Constants.motorFeederFront);

  /** Creates a new FrontFeeder. */
  public FeederFront() {
    motorFeederFront.setInverted(false);
    motorFeederFront.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotor(double speed) {
    motorFeederFront.set(ControlMode.PercentOutput, speed);
  }

  public void setMotorVolts(double volts) {
    motorFeederFront.setVoltage(volts);
  }

  public void stopMotor() {
    motorFeederFront.set(ControlMode.PercentOutput, 0);
  }
}
