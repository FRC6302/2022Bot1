// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecDriveTrain extends SubsystemBase {
  WPI_TalonSRX motorL1;
  WPI_TalonSRX motorL2;
  WPI_TalonSRX motorR1;
  WPI_TalonSRX motorR2;

  //MecanumDrive mecDrive;

  private Encoder encoderL1;
  private Encoder encoderL2;
  private Encoder encoderR1;
  private Encoder encoderR2;

  SimpleMotorFeedforward mecFeedforward = new SimpleMotorFeedforward(
    Constants.ksMecFeedForward, 
    Constants.kvMecFeedForward,
    Constants.kaMecFeedForward);

  ProfiledPIDController pidController = new ProfiledPIDController(Constants.kpMecV, 0, 0, Constants.mecVConstraints);

  //robot width wheel-to-wheel is 0.584 m, length wheel-to-wheel is 0.521 m
  //0.584/2 = 0.292, 0.521/2 = 0.2605
  //use (y, -x) if using normal math graphs, the origin is center of robot
  private final Translation2d frontLeftLocation = new Translation2d(0.2605, 0.292);
  private final Translation2d frontRightLocation = new Translation2d(0.2605, -0.292);
  private final Translation2d backLeftLocation = new Translation2d(-0.2605, 0.292);
  private final Translation2d backRightLocation = new Translation2d(-0.2605, -0.292);

  private final MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, NavX.getGyroRotation2d());

  private final NeutralMode motorMode = NeutralMode.Brake;

  //distance per pulse = pi * (wheel diameter / counts per revolution) / gear reduction
  //rev through bore encoder is 8192 counts per rev? or use 2048 cycles per rev?
  //ctre mag encoder is 4096
  private final double distancePerPulse = (Math.PI * 0.1524 / 2048) / 1; //mec wheel diam is 6 in or 0.1524 m

  private ChassisSpeeds curChassisSpeeds = new ChassisSpeeds();

  /** Creates a new MecDriveTrain. */
  public MecDriveTrain() {
    motorL1 = new WPI_TalonSRX(Constants.motorL1Value);
    motorL2 = new WPI_TalonSRX(Constants.motorL2Value);
    motorR1 = new WPI_TalonSRX(Constants.motorR1Value);
    motorR2 = new WPI_TalonSRX(Constants.motorR2Value);    

    motorL1.configFactoryDefault();
    motorL2.configFactoryDefault();
    motorR1.configFactoryDefault();
    motorR2.configFactoryDefault();

    motorL1.setNeutralMode(motorMode);
    motorL2.setNeutralMode(motorMode);
    motorR1.setNeutralMode(motorMode);
    motorR2.setNeutralMode(motorMode);

    motorL1.setInverted(false);
    motorL2.setInverted(false);
    motorR1.setInverted(true);
    motorR2.setInverted(true);

    
    encoderL1 = new Encoder(Constants.encL1A, Constants.encL1B, true);
    encoderL2 = new Encoder(Constants.encL2A, Constants.encL2B, true);
    encoderR1 = new Encoder(Constants.encR1A, Constants.encR1B, false);
    encoderR2 = new Encoder(Constants.encR2A, Constants.encR2B, false);

    encoderL1.setDistancePerPulse(distancePerPulse);
    encoderL2.setDistancePerPulse(distancePerPulse);
    encoderR1.setDistancePerPulse(distancePerPulse);
    encoderR2.setDistancePerPulse(distancePerPulse);
    
    //mecDrive = new MecanumDrive(motorL1, motorL2, motorR1, motorR2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("enc L1", encoderL1.getDistance());
    SmartDashboard.putNumber("enc L2", encoderL2.getDistance());
    SmartDashboard.putNumber("enc R1", encoderR1.getDistance());
    SmartDashboard.putNumber("enc R2", encoderR2.getDistance());

    updateOdometry();

    //used for converting robot velocity into target-relative velocity
    curChassisSpeeds = kinematics.toChassisSpeeds(getCurrentWheelSpeeds());
    
  }
  
  

  /*public static MecanumDrive.WheelSpeeds driveCartesianIK(double ySpeed, 
  double xSpeed, double zRotation, double gyroAngle) {
    return mecDrive.driveCartesianIK(ySpeed, xSpeed, zRotation, gyroAngle);
  }*/

  public void setMotors(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds =
      kinematics.toWheelSpeeds(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, NavX.getGyroRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    //mecanumDriveWheelSpeeds.desaturate(Constants.maxMecSpeed);
    setSpeeds(mecanumDriveWheelSpeeds); 
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    motorL1.setVoltage(speeds.frontLeftMetersPerSecond);
    motorL2.setVoltage(speeds.rearLeftMetersPerSecond);
    motorR1.setVoltage(speeds.frontRightMetersPerSecond);
    motorR2.setVoltage(speeds.rearRightMetersPerSecond);
    //TODO use pid here somehow with PPMecCommand and also dont use setVoltage?

    SmartDashboard.putNumber("motorL1", speeds.frontLeftMetersPerSecond);
    SmartDashboard.putNumber("motorR1", speeds.frontRightMetersPerSecond);
  }

  public void stopDrive(){
    motorL1.set(ControlMode.PercentOutput, 0);
    motorL2.set(ControlMode.PercentOutput, 0);
    motorR1.set(ControlMode.PercentOutput, 0);
    motorR2.set(ControlMode.PercentOutput, 0);
  }

  double maxMecSpeed = 9;
  public void setMotorsSimple(double xSpeed, double ySpeed, double rotSpeed) {
    motorL1.set(ControlMode.PercentOutput, (xSpeed - ySpeed - rotSpeed) / maxMecSpeed);
    motorL2.set(ControlMode.PercentOutput, (xSpeed + ySpeed - rotSpeed) / maxMecSpeed);
    motorR1.set(ControlMode.PercentOutput, (xSpeed + ySpeed + rotSpeed) / maxMecSpeed);
    motorR2.set(ControlMode.PercentOutput, (xSpeed - ySpeed + rotSpeed) / maxMecSpeed);

    SmartDashboard.putNumber("motorL1", (xSpeed - ySpeed - rotSpeed) / maxMecSpeed);
    SmartDashboard.putNumber("motorR1", (xSpeed + ySpeed + rotSpeed) / maxMecSpeed);

  }

  public void setMotorVolts(MecanumDriveMotorVoltages volts) {
    motorL1.setVoltage(volts.frontLeftVoltage);
    motorL2.setVoltage(volts.rearLeftVoltage);
    motorR1.setVoltage(volts.frontRightVoltage);
    motorR2.setVoltage(volts.rearRightVoltage);
  }

  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      encoderL1.getRate(),
      encoderL2.getRate(),
      encoderR1.getRate(),
      encoderR2.getRate());
  }

   /** Updates the field relative position of the robot. */
   public void updateOdometry() {
    odometry.update(NavX.getGyroRotation2d(), getCurrentWheelSpeeds());
  }


  public void resetEncoders() {
    encoderL1.reset();
    encoderL2.reset();
    encoderR1.reset();
    encoderR2.reset();
  }

  public Encoder getEncoderL1() {
    return encoderL1;
  }

  public Encoder getEncoderL2() {
    return encoderL2;
  }

  public Encoder getEncoderR1() {
    return encoderR1;
  }

  public Encoder getEncoderR2() {
    return encoderR2;
  }

  public MecanumDriveKinematics getMecKinematics() {
    return kinematics;
  }

  public SimpleMotorFeedforward getMecFeedforward() {
    return mecFeedforward;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /*public double getParaVFromFieldRelative(double gyroAngleDeg, double turretAngleDeg) {
    //converts velocities from robot-relative to field-relative then to target-relative
    double vz = curChassisSpeeds.vxMetersPerSecond;
    double vx = curChassisSpeeds.vyMetersPerSecond;
    //use gyro to convert these to field relative speeds

    double theta = Math.toRadians(gyroAngleDeg + turretAngleDeg - 90);

    return vx * Math.cos(theta) + vz * Math.sin(theta);
  }

  public double getPerpVFromFieldRelative(double gyroAngleDeg, double turretAngleDeg) {
    //converts velocities from robot-relative to field-relative then to target-relative
    double vz = curChassisSpeeds.vxMetersPerSecond;
    double vx = curChassisSpeeds.vyMetersPerSecond;
    //use gyro to convert these to field relative speeds 

    double theta = Math.toRadians(gyroAngleDeg + turretAngleDeg - 90);

    return -vx * Math.sin(theta) + vz * Math.cos(theta);
  }*/

  public double getParaV(double turretAngleDeg) {
    //converts from robot-relative velocities directly to target relative velocities
    double vz = curChassisSpeeds.vxMetersPerSecond; //velocity in the foward direction, relative to robot
    double vx = -curChassisSpeeds.vyMetersPerSecond; //velocity towards the right, relative to robot
    double theta = Math.toRadians(90 - turretAngleDeg);

    return vx * Math.cos(theta) + vz * Math.sin(theta);
  }

  public double getPerpV(double turretAngleDeg) {
    //converts from robot-relative velocities directly to target relative velocities
    double vz = curChassisSpeeds.vxMetersPerSecond; //velocity in the foward direction, relative to robot
    double vx = -curChassisSpeeds.vyMetersPerSecond; //velocity towards the right, relative to robot
    double theta = Math.toRadians(90 - turretAngleDeg);

    return -vx * Math.sin(theta) + vz * Math.cos(theta);
  }

  public double getAngV() {
    return Math.toDegrees(curChassisSpeeds.omegaRadiansPerSecond);
  }
}
