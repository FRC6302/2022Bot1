// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
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

  /*private Encoder encL1;
  private Encoder encL2;
  private Encoder encR1;
  private Encoder encR2;*/

  //remeasure
  private final Translation2d frontLeftLocation = new Translation2d(0.5, 0.5);
  private final Translation2d frontRightLocation = new Translation2d(0.5, -0.5);
  private final Translation2d backLeftLocation = new Translation2d(-0.5, 0.5);
  private final Translation2d backRightLocation = new Translation2d(-0.5, -0.5);

  private final MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  //private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, NavX.getGyroRotation2d());

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


   //encL1 = new Encoder(0, 1);
   //encL2 = new Encoder(0, 1);
   //encR1 = new Encoder(0, 1);
   //encR2 = new Encoder(0, 1);
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

  /*public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        encL1.getRate(),
        encL2.getRate(),
        encR1.getRate(),
        encR2.getRate());
  }*/

   /** Updates the field relative position of the robot. */
   public void updateOdometry() {
    //odometry.update(NavX.getGyroRotation2d(), getCurrentState());
  }
}
