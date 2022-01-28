// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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

  //remeasure
  private final Translation2d frontLeftLocation = new Translation2d(0.5, 0.5);
  private final Translation2d frontRightLocation = new Translation2d(0.5, -0.5);
  private final Translation2d backLeftLocation = new Translation2d(-0.5, 0.5);
  private final Translation2d backRightLocation = new Translation2d(-0.5, -0.5);

  private final MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, NavX.getGyroRotation2d());

  private final NeutralMode motorMode = NeutralMode.Coast;


  /** Creates a new MecDriveTrain. */
  public MecDriveTrain() {
    motorL1 = new WPI_TalonSRX(Constants.motorL1Value);
    motorL2 = new WPI_TalonSRX(Constants.motorL2Value);
    motorR1 = new WPI_TalonSRX(Constants.motorR1Value);
    motorR2 = new WPI_TalonSRX(Constants.motorR2Value);

   // mecDrive = new MecanumDrive(motorL1, motorL2, motorR1, motorR2);

   motorL1.setNeutralMode(motorMode);
   motorL2.setNeutralMode(motorMode);
   motorR1.setNeutralMode(motorMode);
   motorR2.setNeutralMode(motorMode);

   motorL1.setInverted(true);
   motorL2.setInverted(true);
   motorR1.setInverted(false);
   motorR2.setInverted(false);


   encoderL1 = new Encoder(Constants.encL1A, Constants.encL1B, false);
   encoderL2 = new Encoder(Constants.encL2A, Constants.encL2B, false);
   encoderR1 = new Encoder(Constants.encR1A, Constants.encR1B, false);
   encoderR2 = new Encoder(Constants.encR2A, Constants.encR2B, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

  private void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    motorL1.setVoltage(speeds.frontLeftMetersPerSecond);
    motorL2.setVoltage(speeds.rearLeftMetersPerSecond);
    motorR1.setVoltage(speeds.frontRightMetersPerSecond);
    motorR2.setVoltage(speeds.rearRightMetersPerSecond);

    SmartDashboard.putNumber("motorL1", speeds.frontLeftMetersPerSecond);
    SmartDashboard.putNumber("motorR1", speeds.frontRightMetersPerSecond);
  }

  public void stopDrive(){
    motorL1.set(ControlMode.PercentOutput, 0);
    motorL2.set(ControlMode.PercentOutput, 0);
    motorR1.set(ControlMode.PercentOutput, 0);
    motorR2.set(ControlMode.PercentOutput, 0);
  }

  double maxMecSpeed = 3;
  public void setMotorsSimple(double xSpeed, double ySpeed, double rotSpeed) {
    motorL1.set(ControlMode.PercentOutput, (xSpeed - ySpeed - rotSpeed) / maxMecSpeed);
    motorL2.set(ControlMode.PercentOutput, (xSpeed + ySpeed - rotSpeed) / maxMecSpeed);
    motorR1.set(ControlMode.PercentOutput, (xSpeed + ySpeed + rotSpeed) / maxMecSpeed);
    motorR2.set(ControlMode.PercentOutput, (xSpeed - ySpeed + rotSpeed) / maxMecSpeed);
  }

  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
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

  public MecanumDriveKinematics getMecKinetimatics() {
    return kinematics;
  }

  public SimpleMotorFeedforward getMecFeedforward() {
    return mecFeedforward;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getParaV() {
    return 0;
  }

  public double getPerpV() {
    return 0;
  }
}
