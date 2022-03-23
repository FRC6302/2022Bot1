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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
  //private PIDController posPIDController = new PIDController(Constants.kpPosTurret, 0, Constants.kdPosTurret);
  //private PIDController velPIDController = new PIDController(Constants.kpVelTurret, 0, 0);
  private ProfiledPIDController posPIDController = new ProfiledPIDController(Constants.kpPosTurret, 0, Constants.kdPosTurret, 
    new Constraints(Constants.maxTurretV, Constants.maxTurretA));
  private ProfiledPIDController velPIDController = new ProfiledPIDController(Constants.kpVelTurret, 0, 0, 
    new Constraints(Constants.maxTurretV, Constants.maxTurretA));

  private SimpleMotorFeedforward simpleFeedforward = new SimpleMotorFeedforward(
    Constants.ksTurret, Constants.kvTurret, Constants.kaTurret);

  double tangentialFeedforward = 0;
  double rotationalFeedforward = 0;

  private double gearReduction = 14;

  private double angleSetpoint = 0;

  //360 degrees
  //8192 cpr for rev through bore encoder
  private double distancePerPulse = (360. / 2048.) / gearReduction;

  //private boolean needsReset = false;
  private boolean resettingForward = false;
  private boolean resettingBackward = false;

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

    //posPIDController.enableContinuousInput(-180, 180);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //tangentialFeedforward = 

    SmartDashboard.putNumber("turret enc pos", getAngle());
    SmartDashboard.putNumber("turret constrained angle", constrainAngle(getAngle()));
    SmartDashboard.putNumber("turret enc vel", getEncVelocity());


    ///resetTrigger.whileActiveOnce(new ResetTurret(this, 0., false), false;)

  }

  public void setMotor(double speed) {
    //motorTurret.set(ControlMode.PercentOutput, speed);
    double volts = speed * 12.5;
    SmartDashboard.putNumber("turret drive volts", volts);
    motorTurret.setVoltage(volts);
  }

  /*public void setMotorPosPID(double tx, double perpV, double distance, double angV){
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
    //SmartDashboard.putNumber("tangent ff", tangentialFeedforward);
    //SmartDashboard.putNumber("rot ff", rotationalFeedforward);
    setVoltageBounded(turretVolts);
  }*/

  public void setMotorPosPID(double posSetpoint, double distance, double perpV, double angV) {
    //double estimatedDistance = Constants.goalLocation.getDistance(robotPose.getTranslation());
    /*feedforwards based on chassis movement to make turret react better.
    We already know which way it needs to turn if the chassis moving a certain direction, so we can help
    it along and then the pid can do the rest*/
    //tangentialFeedforward = Units.radiansToDegrees(perpV / distance);
    //rotationalFeedforward = -angV;

    /*the arctan part gives the angle to the target relative to the field but you have to subtract the pose
    heading (aka gyro angle) so you know what angle to send the turret to relative to the front of the robot*/
    //double posSetpoint = offsetAngle + angleToTarget - gyroAngle;
    posSetpoint = constrainAngle(posSetpoint);

    //constraning both the angles so that turret goes to the closest correct angle instead of going all the way around
    //setVoltageBounded(simpleFeedforward.calculate(posPIDController.calculate(constrainAngle(getAngle()),
      //constrainAngle(posSetpoint)) + tangentialFeedforward + rotationalFeedforward));

    motorTurret.setVoltage(simpleFeedforward.calculate(posPIDController.calculate(getAngle(), posSetpoint)));
  }

  public void setMotorVelPID(double tx, double perpV, double distance, double angV) {
    tangentialFeedforward = Units.radiansToDegrees(perpV / distance);
    rotationalFeedforward = -angV;
    double setpointVel = -tx / 1 /*+ tangentialFeedforward + rotationalFeedforward*/;
    double vel = getEncVelocity();
    double pid = velPIDController.calculate(vel, setpointVel);
    double ff = simpleFeedforward.calculate(setpointVel);
    double turretVolts = ff;
    //SmartDashboard.putNumber("turret pid", pid);
    //SmartDashboard.putNumber("turret ff", ff);
    SmartDashboard.putNumber("turret v", vel);
    SmartDashboard.putNumber("turret setpoint v", setpointVel);
    SmartDashboard.putNumber("turret volts", turretVolts);
    motorTurret.setVoltage(turretVolts);
  }

  public void setMotorVelPID(Pose2d robotPose, double offsetAngle, double perpV, double angV, double gyroYaw) {
    double estimatedDistance = Constants.goalLocation.getDistance(robotPose.getTranslation());
    tangentialFeedforward = Units.radiansToDegrees(perpV / estimatedDistance);
    rotationalFeedforward = -angV;

    double setpointVel = tangentialFeedforward + rotationalFeedforward;

    motorTurret.setVoltage(velPIDController.calculate(getEncVelocity(), setpointVel) 
      + simpleFeedforward.calculate(setpointVel));
  }

  public void turnToAngle(double angleDeg) {
    angleSetpoint = angleDeg;

    //use PID to get to certain encoder values
    motorTurret.setVoltage(simpleFeedforward.calculate(posPIDController.calculate(getAngle(), angleDeg)));
  }
  

  //makes sure that the turret doesn't go past its limits
  private void setVoltageBounded(double volts, double turretAngle) {
    /*if (turretAngle <= Constants.minTurretAngle || resettingForward) {
      resettingForward = true;
      motorTurret.setVoltage(Constants.turretResetVoltage);
      if (turretAngle >= Constants.minTurretAngle + 300) {
        resettingForward = false;
      }
      return;
    }
    else if (turretAngle >= Constants.maxTurretAngle || resettingBackward) {
      resettingBackward = true;
      motorTurret.setVoltage(-Constants.turretResetVoltage);
      if (turretAngle <= Constants.maxTurretAngle - 300) {
        resettingBackward = false;
      }
      return;
    }
    else {
      motorTurret.setVoltage(volts);
    }*/

    if (volts < 0) {
      
    }
  }

  private void setVoltageBounded(double volts) {
    setVoltageBounded(volts, getAngle());
  }

  public double getAngle() {
    //double currAngle = turretEncoder.getDistance() % 360;
    //if (currAngle >= 0)
    return turretEncoder.getDistance();
  }

  //returns the turret angle but between -180 and 180
  public double constrainAngle(double rawAngle) {
    //TODO: test this
    double angle = (rawAngle + 180.0) % 360.0;
    if (angle < 0) {
      angle += 360;
    }
    return angle - 180;
    
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

  /*public boolean getNeedsReset() {
    return needsReset;
  }*/

  public void stopMotor() {
    motorTurret.set(0);
  }

  public void resetEncoder() {
    turretEncoder.reset();
  }

public void missTarget() {
}

}
