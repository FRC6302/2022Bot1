// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final double topkS = 0, topkV = 0, topkA = 0;
  private final double bottomkS = 0, bottomkV = 0, bottomkA = 0;

  private WPI_TalonSRX motorShooterTop = new WPI_TalonSRX(Constants.motorShooterTop);
  private WPI_TalonSRX motorShooterBottom = new WPI_TalonSRX(Constants.motorShooterBottom);

  private Encoder topShooterEncoder = new Encoder(Constants.topShooterEncA, Constants.topShooterEncB, false/*, EncodingType.k4X*/);
  private Encoder bottomShooterEncoder = new Encoder(Constants.bottomShooterEncA, Constants.bottomShooterEncB, false/*, EncodingType.k4X*/);

  private BangBangController bangBangTop = new BangBangController(0.5);
  private BangBangController bangBangBottom = new BangBangController(0.5);

  // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(topkS, topkV, topkA);
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(bottomkS, bottomkV, bottomkA);
  
  /*
    Distance Per Pulse (dpp) calculation explanation:
    distance per pulse is pi * (wheel diameter / counts per revolution) according to Andymark example code
    i think then you divide by gear reduction
    counts per rev is 1024 for our encoder according to Andymark example code
    rev through bore encoder says 8192 counts per rev
    */
  private final double distancePerPulse = (Math.PI * 0.1524 / 8192) / 1; //blue wheel diam is 6 inches or 0.1524 meters
  

  /** Creates a new Shooter. */
  public Shooter() {

    //DO NOT CHANGE OR BANG BANG CONTROL WILL NOT WORK AND SHOOTER WILL BREAK
    motorShooterTop.setNeutralMode(NeutralMode.Coast);
    motorShooterBottom.setNeutralMode(NeutralMode.Coast);

    topShooterEncoder.setDistancePerPulse(distancePerPulse);
    bottomShooterEncoder.setDistancePerPulse(distancePerPulse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("top shooter encoder", topShooterEncoder.getRate());
    SmartDashboard.putNumber("bottom shooter encoder", bottomShooterEncoder.getRate());
  }


  public void shootWithInitialBallVelocity(double paraV, double perpV, double hoodAngle, double offsetAngle, double distance) {
    //offsetAngle = 10 * perpV;
    
    double shotInitV = Math.pow(Math.pow(perpV / Math.tan(offsetAngle) + paraV, 2) 
      + Math.pow(perpV, 2) - 9.807 * Math.pow(distance, 2) * Math.pow(Math.sin(hoodAngle), 2) 
      / (2 * Constants.targetDeltaY * Math.pow(Math.cos(hoodAngle), 2) - 2 * distance 
      * Math.cos(hoodAngle) * Math.sin(hoodAngle)), 0.5);

    setBothMotors(shotInitV / 10);
  }

  public void setBothMotors(double speed) {
    motorShooterTop.set(ControlMode.PercentOutput, speed);
    motorShooterBottom.set(ControlMode.PercentOutput, speed);
  }
  
  public void setBothMotors(double topSpeed, double bottomSpeed) {
    motorShooterTop.set(ControlMode.PercentOutput, topSpeed);
    motorShooterBottom.set(ControlMode.PercentOutput, bottomSpeed);
  }

  /*public void setTopMotor(double speed) {
    motorShooterTop.set(ControlMode.PercentOutput, speed);
  }

  public void setBottomMotor(double speed) {
    motorShooterBottom.set(ControlMode.PercentOutput, speed);
  }*/

  public void setWithBangBang(double desriredSpeed) {
    // Controls a motor with the output of the BangBang controller
    setBothMotors(
      bangBangTop.calculate(topShooterEncoder.getRate(), desriredSpeed), 
      bangBangBottom.calculate(bottomShooterEncoder.getRate(), desriredSpeed));
  }

  public void setWithBangBangAndFeedForward(double desiredSpeed) {
    setBothMotors(
      bangBangTop.calculate(topShooterEncoder.getRate(), desiredSpeed) + 0.9 * topFeedforward.calculate(desiredSpeed),
      bangBangBottom.calculate(bottomShooterEncoder.getRate(), desiredSpeed) + 0.9 * bottomFeedforward.calculate(desiredSpeed));
  }

  
}
