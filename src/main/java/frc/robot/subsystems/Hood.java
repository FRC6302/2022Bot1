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


  private double angleSetpoint = 0;
  private double pidOutput = 0;

  //gear ratio between encoder and hood. The track is 477 and the part that moves is 24 teeth
  double gearReduction = 477.0 / 24.0;

  //360 degrees
  //8192 for rev through bore encoder
  //10 is gear reduction
  //private double distancePerPulse = (360 / 2048) / 10;

  public LinearInterpolator distanceAngleMap;
  private double[][] distanceAngleData = { 
    {1, Units.degreesToRadians(80)}, //{distance in meters, angle in degrees} format
    {3, Units.degreesToRadians(75)}, 
    {5, Units.degreesToRadians(70)},
    {7, Units.degreesToRadians(65)},
    {9, Units.degreesToRadians(60)},
    {11, Units.degreesToRadians(55)},
    {13, Units.degreesToRadians(50)}
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

    //the position factor is unit per rotation and the velocity one is unit per RPM
    //2 pi radians per rotation
    double posFactor = 2 * Math.PI / gearReduction;
    encHood.setPositionConversionFactor(posFactor);
    //60 changes min to sec
    encHood.setVelocityConversionFactor(60 * posFactor);

    //motorHood.setSoftLimit(SoftLimitDirection.kForward, Units.degreesToRadians(70) - Constants.hoodMinimumAngle);
    //motorHood.setSoftLimit(SoftLimitDirection.kReverse, Units.degreesToRadians(20) - Constants.hoodMinimumAngle);

    //motorHood.enableSoftLimit(SoftLimitDirection.kForward, true);
    //motorHood.enableSoftLimit(SoftLimitDirection.kReverse, true);

    motorHood.burnFlash();

    distanceAngleMap = new LinearInterpolator(distanceAngleData);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hood enc pos deg", getAngleDeg());
  }
  
  //add boolean argument for whether scoring or missing?
  public void setMotorPosPID(double distance, double paraV) { 
    paraFeedforward = paraV / distance; //this is just a guess
    double desiredAngle = distanceAngleMap.getInterpolatedValue(distance);
    double pidOutput = pidController.calculate(getAngleRad(), desiredAngle);

    motorHood.setVoltage(feedforward.calculate(desiredAngle, pidOutput + paraFeedforward));

    //angleSetpoint = desiredAngle;
  }

  public void setMotorVelPID(double distance, double paraV) {
    paraFeedforward = paraV / distance; //this is just a guess //radians
    double desiredAngle = distanceAngleMap.getInterpolatedValue(distance);
    double setpointV = pidController.calculate(getAngleRad(), desiredAngle) + paraFeedforward;
    //this would need two different PID controllers? Waste of time
    motorHood.setVoltage(pidController.calculate(getEncVelocityRad(), setpointV) 
      + feedforward.calculate(getAngleRad(), setpointV));

    //oangleSetpoint = desiredAngle;
  }

  public void setAngle(double angleRad) {
    //angleSetpoint = angleRad;

    //use PID to get to certain encoder values
    pidOutput = pidController.calculate(getAngleRad(), angleRad);
    motorHood.setVoltage(feedforward.calculate(getAngleRad(), pidOutput));
  }

  public void setMotor(double speed) {
    //motorHood.set(ControlMode.PercentOutput, speed);
    motorHood.set(speed);
  }

  public double getAngleDeg() {
    return Units.radiansToDegrees(getAngleRad());
  }

  public double getAngleRad() {
    return encHood.getPosition() + Constants.hoodMinimumAngle;
  }

  /*public double getAngleSetpoint() {
    return angleSetpoint;
  }*/

  public double getEncVelocityDeg() {
    return Units.radiansToDegrees(getEncVelocityRad());
  }

  public double getEncVelocityRad() {
    return encHood.getVelocity();
  }

  public void stopMotor() {
    motorHood.set(0);
  }
}
