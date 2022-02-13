// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.LinearInterpolator;

public class Shooter extends SubsystemBase {
  private final double topkS = 0, topkV = 0, topkA = 0;
  private final double bottomkS = 0, bottomkV = 0, bottomkA = 0;

  //private WPI_TalonSRX motorShooterTop = new WPI_TalonSRX(Constants.motorShooterTop);
  //private WPI_TalonSRX motorShooterBottom = new WPI_TalonSRX(Constants.motorShooterBottom);

  private CANSparkMax motorShooterTop = new CANSparkMax(Constants.motorShooterTop, MotorType.kBrushless);
  private CANSparkMax motorShooterBottom = new CANSparkMax(Constants.motorShooterBottom, MotorType.kBrushless);

  /*private Encoder topShooterEncoder = new Encoder(Constants.topShooterEncA, Constants.topShooterEncB, 
    false/*, EncodingType.k4X);
  private Encoder bottomShooterEncoder = new Encoder(Constants.bottomShooterEncA, Constants.bottomShooterEncB, 
    true/*, EncodingType.k4X);*/

  private RelativeEncoder topShooterEncoder;
  private RelativeEncoder bottomShooterEncoder;

  private BangBangController bangBangTop = new BangBangController(0.05);
  private BangBangController bangBangBottom = new BangBangController(0.05);

  // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(topkS, topkV, topkA);
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(bottomkS, bottomkV, bottomkA);
  
  
  //distance per pulse = pi * (wheel diameter / counts per revolution) / gear reduction between encoder and shaft
  //rev through bore encoder is 8192 counts per rev?
  //ctre mag encoder is 4096
  private final double distancePerPulse = Math.PI * 0.1524; // / 4096) / 1; //blue wheel diam is 6 inches or 0.1524 meters

  private LinearInterpolator distanceVelocityMap;
  private double[][] distanceVelocityData = { 
    {1.0, 12}, //{distance in meters, velocity in m/s} format
    {3.0, 14}, 
    {10, 16} };

  private LinearInterpolator distanceTimeMap; 
  private double[][] distanceTimeData = { 
      {1.0, 2}, //{distance in meters, time in sec} format
      {3.0, 3}, 
      {10, 5} };
  
  /** Creates a new Shooter. */
  public Shooter() {

    /*motorShooterTop.configFactoryDefault();
    motorShooterBottom.configFactoryDefault();

    //DO NOT CHANGE OR BANG BANG CONTROL WILL NOT WORK AND SHOOTER WILL BREAK
    motorShooterTop.setNeutralMode(NeutralMode.Coast);
    motorShooterBottom.setNeutralMode(NeutralMode.Coast);

    //motorShooterTop.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    //motorShooterBottom.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    motorShooterTop.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    motorShooterBottom.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    motorShooterTop.setSensorPhase(false);
    motorShooterBottom.setSensorPhase(false);
    
    motorShooterTop.setInverted(false);
    motorShooterBottom.setInverted(true);*/
    
    //topShooterEncoder.setDistancePerPulse(distancePerPulse);
    //bottomShooterEncoder.setDistancePerPulse(distancePerPulse);

    //motorShooterBottom.configAllSettings(new TalonSRXConfiguration());

    motorShooterTop.restoreFactoryDefaults();
    motorShooterBottom.restoreFactoryDefaults();

    motorShooterTop.setInverted(false);
    motorShooterBottom.setInverted(true);

    //DO NOT CHANGE OR BANG BANG CONTROL WILL NOT WORK AND SHOOTER WILL BREAK
    motorShooterTop.setIdleMode(IdleMode.kCoast);
    motorShooterBottom.setIdleMode(IdleMode.kCoast);

    /*this line only works if using neo built-in encoders, otherwise you need arguments that describe
    the encoder that you are using instead*/
    topShooterEncoder = motorShooterTop.getEncoder();
    bottomShooterEncoder = motorShooterTop.getEncoder();

    topShooterEncoder.setInverted(false);
    bottomShooterEncoder.setInverted(false);

    topShooterEncoder.setPositionConversionFactor(distancePerPulse);
    bottomShooterEncoder.setPositionConversionFactor(distancePerPulse);

    topShooterEncoder.setVelocityConversionFactor(distancePerPulse);
    bottomShooterEncoder.setVelocityConversionFactor(distancePerPulse);

    //saves the settings
    motorShooterTop.burnFlash();
    motorShooterBottom.burnFlash();

    distanceVelocityMap = new LinearInterpolator(distanceVelocityData);
    distanceTimeMap = new LinearInterpolator(distanceTimeData);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("top shooter encoder", topShooterEncoder.getRate());
    //SmartDashboard.putNumber("bottom shooter encoder", bottomShooterEncoder.getRate());

    SmartDashboard.putNumber("top shooter encoder", getTopShooterEncRate());
    SmartDashboard.putNumber("bottom shooter encoder", getBottomShooterEncRate());
  }


  public void shootWithInitialBallVelocity(double paraV, double perpV, double hoodAngle, double offsetAngle, double distance) {
    //offsetAngle = 10 * perpV;
    
    double shotInitV = Math.pow(Math.pow(perpV / Math.tan(offsetAngle) + paraV, 2) 
      + Math.pow(perpV, 2) - 9.807 * Math.pow(distance, 2) * Math.pow(Math.sin(hoodAngle), 2) 
      / (2 * Constants.targetDeltaY * Math.pow(Math.cos(hoodAngle), 2) - 2 * distance 
      * Math.cos(hoodAngle) * Math.sin(hoodAngle)), 0.5);

    setMotors(shotInitV / 10);
  }

  public void setMotorsPosPID(double distance, double perpV, double paraV) {
    double setpointV = distanceVelocityMap.getInterpolatedValue(distance);
    
    //break setpointV into component parts
    //add perpV and paraV to their respective components
    //combine components into new setpointV
  }

  public void setMotors(double speed) {
    //motorShooterTop.set(ControlMode.PercentOutput, speed);
    //motorShooterBottom.set(ControlMode.PercentOutput, speed);

    motorShooterTop.set(speed);
    motorShooterBottom.set(speed);
  }
  
  public void setMotors(double topSpeed, double bottomSpeed) {
    //motorShooterTop.set(ControlMode.PercentOutput, topSpeed);
    //motorShooterBottom.set(ControlMode.PercentOutput, bottomSpeed);

    motorShooterTop.set(topSpeed);
    motorShooterBottom.set(bottomSpeed);
  }

  /*public void setTopMotor(double speed) {
    motorShooterTop.set(ControlMode.PercentOutput, speed);
  }

  public void setBottomMotor(double speed) {
    motorShooterBottom.set(ControlMode.PercentOutput, speed);
  }*/

  public double getTopShooterEncRate() {
    //multiplying by 10 because to turn 100 ms to 1 sec because it reports with per 100 ms units
    //return 10.0 * motorShooterTop.getSelectedSensorVelocity() * distancePerPulse;

    return topShooterEncoder.getVelocity();
  }

  public double getBottomShooterEncRate() {
    //return 10.0 * motorShooterBottom.getSelectedSensorVelocity() * distancePerPulse;

    return bottomShooterEncoder.getVelocity();
  }

  public void setWithBangBang(double desiredSpeed) {
    // Controls a motor with the output of the BangBang controller
    setMotors(
      bangBangTop.calculate(getTopShooterEncRate(), desiredSpeed), 
      bangBangBottom.calculate(getBottomShooterEncRate(), desiredSpeed));
  }

  public void setWithBangBang(double desiredTop, double desiredBottom) {
    // Controls a motor with the output of the BangBang controller
    setMotors(
      bangBangTop.calculate(getTopShooterEncRate(), desiredTop), 
      bangBangBottom.calculate(getBottomShooterEncRate(), desiredBottom));
  }

  public void setWithBangBangAndFeedForward(double desiredSpeed) {
    setMotors(
      bangBangTop.calculate(getTopShooterEncRate(), desiredSpeed) 
        + 0.9 * topFeedforward.calculate(desiredSpeed),
      bangBangBottom.calculate(getBottomShooterEncRate(), desiredSpeed) 
        + 0.9 * bottomFeedforward.calculate(desiredSpeed));
  }

  public void setWithBangBangAndFeedForward(double desiredTop, double desiredBottom) {
    setMotors(
      bangBangTop.calculate(getTopShooterEncRate(), desiredTop) 
        + 0.9 * topFeedforward.calculate(desiredTop),
      bangBangBottom.calculate(getBottomShooterEncRate(), desiredBottom) 
        + 0.9 * bottomFeedforward.calculate(desiredBottom));
  }

  public void stop() {
    setMotors(0);
  }
  
}
