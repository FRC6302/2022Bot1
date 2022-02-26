// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private WPI_TalonSRX motorTurret = new WPI_TalonSRX(Constants.motorTurret);

  private Encoder turretEncoder;

  //private ProfiledPIDController pidController = new ProfiledPIDController(Constants.kpTurret, 0, 0, Constants.turretConstraints);
  private PIDController pidController = new PIDController(Constants.kpTurret, 0, 0);

  private SimpleMotorFeedforward simpleFeedforward = new SimpleMotorFeedforward(
    Constants.ksTurret, Constants.kvTurret, Constants.kaTurret);

  double tangentialFeedforward = 0;
  double rotationalFeedforward = 0;

  private double gearReduction = 14;

  private double angleSetpoint = 0;

  //360 degrees
  //8192 cpr for rev through bore encoder
  private double distancePerPulse = (360. / 2048.) / gearReduction;

  /** Creates a new Turret. */
  public Turret() {
    motorTurret.setNeutralMode(NeutralMode.Brake);
    motorTurret.setInverted(false);

    //motorTurret.configForwardSoftLimitThreshold(720);
    //motorTurret.configForwardSoftLimitEnable(true);
    //motorTurret.configReverseSoftLimitThreshold(720);
    //motorTurret.configReverseSoftLimitEnable(true);

    turretEncoder = new Encoder(Constants.encTurretA, Constants.encTurretB, false);
    turretEncoder.setDistancePerPulse(distancePerPulse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //tangentialFeedforward = 

    SmartDashboard.putNumber("turret enc pos", getAngle());
    SmartDashboard.putNumber("turret enc vel", getEncVelocity());

  }

  public void setMotor(double speed) {
    //motorTurret.set(ControlMode.PercentOutput, speed);
    double volts = speed * 12;
    SmartDashboard.putNumber("turret drive volts", volts);
    motorTurret.setVoltage(volts);
  }

  /*public void setMotor(double setpointV, double perpV, double distance, double angV){
    tangentialFeedforward = perpV / distance;
    rotationalFeedforward = -angV;
    setpointV = setpointV + tangentialFeedforward + rotationalFeedforward;

    motorTurret.setVoltage(pidController.calculate(getEncVelocity(), setpointV) + simpleFeedforward.calculate(setpointV));
  }*/

  public void setMotorPosPID(double tx, double perpV, double distance, double angV){
    tangentialFeedforward = perpV / distance;
    rotationalFeedforward = -angV;
    //double angle = getAngle();
    //double setpoint = angle - tx;
    double pidOutput = pidController.calculate(tx, 0);
    double turretVolts = simpleFeedforward.calculate(
      pidOutput/* + tangentialFeedforward + rotationalFeedforward*/);
    SmartDashboard.putNumber("turret volts", turretVolts);
    SmartDashboard.putNumber("pid turret", pidOutput);
    motorTurret.setVoltage(turretVolts);
  }

  public void setMotorPosPID(Pose2d robotPose, double perpV, double angV) {
    double estimatedDistance = Constants.goalLocation.getDistance(robotPose.getTranslation());
    tangentialFeedforward = perpV / estimatedDistance;
    rotationalFeedforward = -angV;

    double posSetpoint = 0;

    motorTurret.setVoltage(simpleFeedforward.calculate(pidController.calculate(getAngle(), posSetpoint)
      + tangentialFeedforward + rotationalFeedforward));
  }
  
  

  public void setMotorVelPID(double tx, double perpV, double distance, double angV) {
    

    double setpointVel = -tx / 1 + tangentialFeedforward + rotationalFeedforward;
    
    double pid = pidController.calculate(getEncVelocity(), setpointVel);
    double ff = simpleFeedforward.calculate(setpointVel);
    double turretVolts = pid + ff;
    SmartDashboard.putNumber("turret pid", pid);
    SmartDashboard.putNumber("turret ff", ff);
    SmartDashboard.putNumber("turret volts", turretVolts);
    motorTurret.setVoltage(turretVolts);
  }

  public void setMotorVelPID(Pose2d robotPose, double perpV, double angV) {
    double estimatedDistance = Constants.goalLocation.getDistance(robotPose.getTranslation());
    tangentialFeedforward = perpV / estimatedDistance;
    rotationalFeedforward = -angV;

    double setpointVel = robotPose.getRotation().getDegrees();

    motorTurret.setVoltage(pidController.calculate(getEncVelocity(), setpointVel) 
      + simpleFeedforward.calculate(setpointVel));
  }

  public void setAngle(double angleDeg) {
    angleSetpoint = angleDeg;

    //use PID to get to certain encoder values
    motorTurret.setVoltage(pidController.calculate(getAngle(), angleDeg));
  }

  public double getAngle() {
    return turretEncoder.getDistance();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  public double getAngleSetpoint() {
    return angleSetpoint;
  }

  public double getEncVelocity() {
    return turretEncoder.getRate();
  }

  public void stopMotor() {
    motorTurret.set(0);
  }

}
