// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private WPI_TalonSRX motorTurret = new WPI_TalonSRX(Constants.motorTurret);

  private Encoder turretEncoder = new Encoder(Constants.encTurretA, Constants.encTurretB, false);

  private ProfiledPIDController pidController = new ProfiledPIDController(Constants.kpTurret, 0, 0, Constants.turretConstraints);

  private SimpleMotorFeedforward simpleFeedforward = new SimpleMotorFeedforward(
    Constants.ksTurret, Constants.kvTurret, Constants.kaTurret);

  double tangentialFeedforward = 0;
  double rotationalFeedforward = 0;

  private double gearReduction = 10;

  private double angleSetpoint = 0;

  //360 degrees
  //8192 for rev through bore encoder
  private double distancePerPulse = (360 / 8192) / gearReduction;

  /** Creates a new Turret. */
  public Turret() {
    motorTurret.setNeutralMode(NeutralMode.Brake);
    motorTurret.setInverted(false);

    turretEncoder.setDistancePerPulse(distancePerPulse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //tangentialFeedforward = 

  }

  public void setMotor(double speed) {
    motorTurret.set(ControlMode.PercentOutput, speed);
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

    motorTurret.setVoltage(simpleFeedforward.calculate(
      pidOutput + tangentialFeedforward + rotationalFeedforward));
  }

  public void setMotorVelPID(double tx, double perpV, double distance, double angV) {
    tangentialFeedforward = perpV / distance;
    rotationalFeedforward = -angV;

    double setpointV = tx / 10 + tangentialFeedforward + rotationalFeedforward;
    motorTurret.setVoltage(pidController.calculate(getEncVelocity(), setpointV) 
      + simpleFeedforward.calculate(setpointV));
  }

  public void setAngle(double angleDeg) {
    angleSetpoint = angleDeg;

    //use PID to get to certain encoder values
    motorTurret.setVoltage(pidController.calculate(getAngle(), angleDeg));
  }

  public double getAngle() {
    return turretEncoder.getDistance();
  }

  public double getAngleSetpoint() {
    return angleSetpoint;
  }

  public double getEncVelocity() {
    return turretEncoder.getRate();
  }

}
