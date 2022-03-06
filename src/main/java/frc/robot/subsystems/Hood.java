// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.LinearInterpolator;

public class Hood extends SubsystemBase {
  //private WPI_TalonSRX motorHood = new WPI_TalonSRX(Constants.motorHood);
  //private Encoder hoodEncoder = new Encoder(Constants.encHoodA, Constants.encHoodB, false);

  private CANSparkMax motorHood = new CANSparkMax(Constants.motorHood, MotorType.kBrushless);
  private RelativeEncoder encHood;
  

  private ProfiledPIDController pidController = new ProfiledPIDController(Constants.kpHood, 0, 0, 
    new Constraints(Constants.maxHoodV, Constants.maxHoodA));

  /*private SimpleMotorFeedforward simpleFeedforward = new SimpleMotorFeedforward(
    Constants.ksHood, Constants.kvHood, Constants.kaHood);*/

  //hood works the same as an would in this case. Just corrects for gravity pulling the hood/arm down
  private ArmFeedforward feedforward = new ArmFeedforward(Constants.ksHood, 
    Constants.kgHood, Constants.kvHood, Constants.kaHood);

  private double paraFeedforward = 0;

  private double gearReduction = 10;

  private double angleSetpoint = 0;
  private double pidOutput = 0;

  //360 degrees
  //8192 for rev through bore encoder
  private double distancePerPulse = (360 / 2048) / gearReduction;

  public LinearInterpolator distanceAngleMap;
  private double[][] distanceAngleData = { 
    {1.0, 80.0}, //{distance in meters, angle in degrees} format
    {3.0, 65.0}, 
    {10, 50.0} 
  };

  /** Creates a new Hood. */
  public Hood() {
    /*motorHood.setNeutralMode(NeutralMode.Brake);
    motorHood.setInverted(false);
    
    hoodEncoder.setDistancePerPulse(distancePerPulse);*/

    motorHood.restoreFactoryDefaults();
    motorHood.setIdleMode(IdleMode.kBrake);
    motorHood.setInverted(false);

    encHood = motorHood.getEncoder();
    //the position one is unit per rotation and the velocity one is unit per RPM
    encHood.setPositionConversionFactor(360);
    encHood.setVelocityConversionFactor(360 * 60);

    motorHood.setSoftLimit(SoftLimitDirection.kForward, 90);
    motorHood.setSoftLimit(SoftLimitDirection.kReverse, 0);

    motorHood.enableSoftLimit(SoftLimitDirection.kForward, true);
    motorHood.enableSoftLimit(SoftLimitDirection.kReverse, true);

    motorHood.burnFlash();

    distanceAngleMap = new LinearInterpolator(distanceAngleData);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hood enc pos", getAngle());
  }
  
  //add boolean argument for whether scoring or missing?
  public void setMotorPosPID(double distance, double paraV) { 
    paraFeedforward = paraV / distance; //this is just a guess
    double desiredAngle = distanceAngleMap.getInterpolatedValue(distance);
    double pidOutput = pidController.calculate(getAngle(), desiredAngle);

    motorHood.setVoltage(feedforward.calculate(getAngleRad(), pidOutput + paraFeedforward));

    angleSetpoint = desiredAngle;
  }

  public void setMotorVelPID(double distance, double paraV) {
    paraFeedforward = paraV / distance; //this is just a guess
    double desiredAngle = distanceAngleMap.getInterpolatedValue(distance);
    double setpointV = pidController.calculate(getAngle(), desiredAngle) + paraFeedforward;
    //this would need two different PID controllers? Waste of time
    motorHood.setVoltage(pidController.calculate(getEncVelocity(), setpointV) 
      + feedforward.calculate(getAngleRad(), setpointV));

    angleSetpoint = desiredAngle;
  }

  public void setAngle(double angleDeg) {
    angleSetpoint = angleDeg;

    //use PID to get to certain encoder values
    pidOutput = pidController.calculate(getAngle(), angleDeg);
    motorHood.setVoltage(feedforward.calculate(getAngleRad(), pidOutput));
  }

  public void setMotor(double speed) {
    //motorHood.set(ControlMode.PercentOutput, speed);
    motorHood.set(speed);
  }

  public double getAngle() {
    return encHood.getPosition() + Constants.hoodMinimumAngle;
  }

  public double getAngleRad() {
    return Units.degreesToRadians(getAngle());
  }

  public double getAngleSetpoint() {
    return angleSetpoint;
  }

  public double getEncVelocity() {
    return encHood.getVelocity();
  }

  public double getEncVelocityRad() {
    return Units.degreesToRadians(getEncVelocity());
  }

  public void stopMotor() {
    motorHood.set(0);
  }
}
