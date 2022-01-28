// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  WPI_TalonSRX motorHood = new WPI_TalonSRX(Constants.motorHood);

  private Encoder hoodEncoder = new Encoder(Constants.encHoodA, Constants.encHoodB, false);

  private double gearReduction = 10;

  //360 degrees
  //8192 for rev through bore encoder
  private double distancePerPulse = (360 / 8192) / gearReduction;

  /** Creates a new Hood. */
  public Hood() {
    motorHood.setNeutralMode(NeutralMode.Brake);
    motorHood.setInverted(false);
    
    hoodEncoder.setDistancePerPulse(distancePerPulse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHoodAngle(double angle) {
    //use PID to get to certain encoder values
  }

  public void setMotor(double speed) {
    motorHood.set(ControlMode.PercentOutput, speed);
  }

  public double getAngle() {
    return hoodEncoder.getDistance();
  }
}
