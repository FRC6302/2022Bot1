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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.ResetTurret;

public class Turret extends SubsystemBase {
  private WPI_TalonSRX motorTurret = new WPI_TalonSRX(Constants.motorTurret);

  private Encoder turretEncoder;

  //private ProfiledPIDController pidController = new ProfiledPIDController(Constants.kpTurret, 0, 0, Constants.turretConstraints);
  private PIDController posPIDController = new PIDController(Constants.kpPosTurret, 0, 0);
  private PIDController velPIDController = new PIDController(Constants.kpVelTurret, 0, 0);

  private SimpleMotorFeedforward simpleFeedforward = new SimpleMotorFeedforward(
    Constants.ksTurret, Constants.kvTurret, Constants.kaTurret);

  double tangentialFeedforward = 0;
  double rotationalFeedforward = 0;

  private double gearReduction = 14;

  private double angleSetpoint = 0;

  //360 degrees
  //8192 cpr for rev through bore encoder
  private double distancePerPulse = (360. / 2048.) / gearReduction;

  private boolean needsReset = false;

  //can you say "this" before constructor runs???
  //private Trigger resetTrigger = new Trigger(this::getNeedsReset);

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

    ///resetTrigger.whileActiveOnce(new ResetTurret(this, 0., false), false;)

  }

  public void setMotor(double speed) {
    //motorTurret.set(ControlMode.PercentOutput, speed);
    double volts = speed * 12.5;
    SmartDashboard.putNumber("turret drive volts", volts);
    motorTurret.setVoltage(volts);
  }

  public void setMotorPosPID(double tx, double perpV, double distance, double angV){
    tangentialFeedforward = Units.radiansToDegrees(perpV / distance);
    rotationalFeedforward = -angV;
    //double angle = getAngle();
    //double setpoint = angle - tx;
    double pidOutput = posPIDController.calculate(tx, 0);
    //double pidOutput = 0;
    double turretVolts = simpleFeedforward.calculate(
      pidOutput + tangentialFeedforward + rotationalFeedforward);
    SmartDashboard.putNumber("turret volts", turretVolts);
    SmartDashboard.putNumber("pid turret", pidOutput);
    SmartDashboard.putNumber("tangent ff", tangentialFeedforward);
    SmartDashboard.putNumber("rot ff", rotationalFeedforward);
    motorTurret.setVoltage(turretVolts);
  }

  public void setMotorPosPID(Pose2d robotPose, double offsetAngle, double perpV, double angV) {
    double estimatedDistance = Constants.goalLocation.getDistance(robotPose.getTranslation());
    tangentialFeedforward = perpV / estimatedDistance;
    rotationalFeedforward = -angV;

    double posSetpoint = offsetAngle + Math.atan2(robotPose.getY() - Constants.goalLocation.getY(), robotPose.getX() - Constants.goalLocation.getX());

    motorTurret.setVoltage(simpleFeedforward.calculate(posPIDController.calculate(getAngle(), posSetpoint)
      + tangentialFeedforward + rotationalFeedforward));
  }

  public void setMotorVelPID(double tx, double perpV, double distance, double angV) {
    tangentialFeedforward = perpV / distance;
    rotationalFeedforward = -angV;
    double setpointVel = -tx / 1 + tangentialFeedforward + rotationalFeedforward;
    
    double pid = velPIDController.calculate(getEncVelocity(), setpointVel);
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

    motorTurret.setVoltage(velPIDController.calculate(getEncVelocity(), setpointVel) 
      + simpleFeedforward.calculate(setpointVel));
  }

  public void turnToAngle(double angleDeg) {
    angleSetpoint = angleDeg;

    //use PID to get to certain encoder values
    motorTurret.setVoltage(simpleFeedforward.calculate(posPIDController.calculate(getAngle(), angleDeg)));
  }
  

  //makes sure that the turret go past its limits
  private void setVoltageBounded(double volts, double turretAngle) {
    if (getAngle() <= Constants.minTurretAngle) {
      motorTurret.setVoltage(-1);
      return;
    }
    else if (getAngle() >= Constants.maxTurretAngle) {
      motorTurret.setVoltage(1);
    }
    else {
      motorTurret.setVoltage(volts);
    }
  }

  private void setVoltageBounded(double volts) {
    double turretAngle = getAngle();
    setVoltageBounded(volts, turretAngle);
  }

  public double getAngle() {
    //double currAngle = turretEncoder.getDistance() % 360;
    //if (currAngle >= 0)
    return turretEncoder.getDistance();
  }

  //returns the angle but between -360 and 360 ???
  public double getAngleBounded() {
    //TODO: test this
    return turretEncoder.getDistance() % 360;
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

  public boolean getNeedsReset() {
    return needsReset;
  }

  public void stopMotor() {
    motorTurret.set(0);
  }

  public void resetEncoder() {
    turretEncoder.reset();
  }

}
