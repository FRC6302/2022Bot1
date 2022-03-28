// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeders extends SubsystemBase {
  private WPI_TalonSRX motorFeederFront = new WPI_TalonSRX(Constants.motorFeederFront);
  private WPI_TalonSRX motorFeederMiddle = new WPI_TalonSRX(Constants.motorFeederMiddle);


  /** Creates a new Feeders. */
  public Feeders() {
    motorFeederFront.setInverted(false);
    motorFeederMiddle.setInverted(false);

    motorFeederFront.setNeutralMode(NeutralMode.Brake);
    motorFeederMiddle.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBothMotors(double frontSpeed, double middleSpeed) {
    motorFeederFront.set(ControlMode.PercentOutput, frontSpeed);
    motorFeederMiddle.set(ControlMode.PercentOutput, middleSpeed);
  }

  public void setBothMotors() {
    motorFeederFront.setVoltage(Constants.feederFrontDefaultVolts);
    motorFeederMiddle.setVoltage(Constants.feederMiddleDefaultVolts);
  }

  public void setBothMotorVolts(double frontVolts, double middleVolts) {
    motorFeederFront.setVoltage(frontVolts);
    motorFeederMiddle.setVoltage(middleVolts);
  }

  public void stopBothMotors() {
    motorFeederFront.set(ControlMode.PercentOutput, 0);
    motorFeederMiddle.set(ControlMode.PercentOutput, 0);
  }



  public void setFrontMotor(double speed) {
    motorFeederFront.set(ControlMode.PercentOutput, speed);
  }

  public void setFrontMotorVolts(double volts) {
    motorFeederFront.setVoltage(volts);
  }

  public void stopFrontMotor() {
    motorFeederFront.set(ControlMode.PercentOutput, 0);
  }

  

  public void setMiddleMotor(double speed) {
    motorFeederMiddle.set(ControlMode.PercentOutput, speed);
  }

  public void setMiddleMotorVolts(double volts) {
    motorFeederMiddle.setVoltage(volts);
  }

  public void stopMiddleMotor() {
    motorFeederMiddle.set(ControlMode.PercentOutput, 0);
  }
}
