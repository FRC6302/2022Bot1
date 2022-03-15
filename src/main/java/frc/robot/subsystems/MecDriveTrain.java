// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.library.VisionPoseEstimation;

public class MecDriveTrain extends SubsystemBase {
  /*WPI_TalonSRX motorL1;
  WPI_TalonSRX motorL2;
  WPI_TalonSRX motorR1;
  WPI_TalonSRX motorR2;*/

  private CANSparkMax motorL1;
  private CANSparkMax motorL2;
  private CANSparkMax motorR1;
  private CANSparkMax motorR2; 

  //MecanumDrive mecDrive;

  private Encoder encoderL1;
  private Encoder encoderL2;
  private Encoder encoderR1;
  private Encoder encoderR2;

  private RelativeEncoder encTest;

  SimpleMotorFeedforward simpleFeedforward = new SimpleMotorFeedforward(
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

  //private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, NavX.getGyroRotation2d());

  private final MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(
    NavX.getGyroRotation2d(),
    new Pose2d(),
    kinematics,
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(Units.degreesToRadians(15)), //wheels slip a lot so their readings are less accurate
    VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(3)));

  //private final NeutralMode motorMode = NeutralMode.Brake; //for talons

  //distance per pulse = pi * (wheel diameter / counts per revolution) / gear reduction
  //rev through bore encoder is 8192 counts per rev? or use 2048 cycles per rev?
  //ctre mag encoder is 4096
  private final double distancePerPulse = (Math.PI * 0.1524 / 2048) / 1; //mec wheel diam is 6 in or 0.1524 m

  private ChassisSpeeds curChassisSpeeds = new ChassisSpeeds();

  private Field2d field = new Field2d();

  /** Creates a new MecDriveTrain. */
  public MecDriveTrain() {
    /*motorL1 = new WPI_TalonSRX(Constants.motorL1Value);
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
    motorR2.setInverted(true);*/

    motorL1 = new CANSparkMax(Constants.motorL1Value, MotorType.kBrushless);
    motorL2 = new CANSparkMax(Constants.motorL2Value, MotorType.kBrushless);
    motorR1 = new CANSparkMax(Constants.motorR1Value, MotorType.kBrushless);
    motorR2 = new CANSparkMax(Constants.motorR2Value, MotorType.kBrushless);

    motorL1.restoreFactoryDefaults();
    motorL2.restoreFactoryDefaults();
    motorR1.restoreFactoryDefaults();
    motorR2.restoreFactoryDefaults();

    motorL1.setInverted(false);
    motorL2.setInverted(false);
    motorR1.setInverted(true);
    motorR2.setInverted(true);

    motorL1.setIdleMode(IdleMode.kBrake);
    motorL2.setIdleMode(IdleMode.kBrake);
    motorR1.setIdleMode(IdleMode.kBrake);
    motorR2.setIdleMode(IdleMode.kBrake);

    //encTest = motorL2.getAlternateEncoder(Type.kQuadrature, 2048);
    encTest = motorL2.getAlternateEncoder(2048);
    //motorL1.getEncoder(encoderType, countsPerRev)
    //encTest.setPositionConversionFactor(1);
    //encTest.setVelocityConversionFactor(1);
    
    motorL1.burnFlash();
    motorL2.burnFlash();
    motorR1.burnFlash();
    motorR2.burnFlash();


    encoderL1 = new Encoder(Constants.encL1A, Constants.encL1B, true, CounterBase.EncodingType.k4X);
    encoderL2 = new Encoder(Constants.encL2A, Constants.encL2B, true, CounterBase.EncodingType.k4X);
    encoderR1 = new Encoder(Constants.encR1A, Constants.encR1B, false, CounterBase.EncodingType.k4X);
    encoderR2 = new Encoder(Constants.encR2A, Constants.encR2B, false, CounterBase.EncodingType.k4X);



    encoderL1.setDistancePerPulse(distancePerPulse);
    encoderL2.setDistancePerPulse(distancePerPulse);
    encoderR1.setDistancePerPulse(distancePerPulse);
    encoderR2.setDistancePerPulse(distancePerPulse);

    
    
    //mecDrive = new MecanumDrive(motorL1, motorL2, motorR1, motorR2);

    //use this to show balls on the field?
    //field.getObject("red balls").setPose(new Pose2d());
    

    SmartDashboard.putData("field", field);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("enc L1", encoderL1.getDistance());
    SmartDashboard.putNumber("enc L2", encoderL2.getDistance());
    SmartDashboard.putNumber("enc R1", encoderR1.getDistance());
    SmartDashboard.putNumber("enc R2", encoderR2.getDistance());

    //SmartDashboard.putNumber("test enc pos", encTest.getPosition());
    //SmartDashboard.putNumber("test enc vel", encTest.getVelocity());

    updateOdometry();

    SmartDashboard.putNumber("pose x", getPoseEstimate().getX());
    SmartDashboard.putNumber("pose y", getPoseEstimate().getY());
    
    field.setRobotPose(getPoseEstimate());

    //SmartDashboard.putNumber("mecanum vx", getVx());
    //SmartDashboard.putNumber("mecanum vy", getVy());
    //SmartDashboard.putNumber("mecanum ang v", getAngV());

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
    /*motorL1.setVoltage(simpleFeedforward.calculate(speeds.frontLeftMetersPerSecond) 
      + pidController.calculate(getEncL1Rate(), speeds.frontLeftMetersPerSecond));
    motorL2.setVoltage(simpleFeedforward.calculate(speeds.rearLeftMetersPerSecond)
      + pidController.calculate(getEncL2Rate(), speeds.rearLeftMetersPerSecond));
    motorR1.setVoltage(simpleFeedforward.calculate(speeds.frontRightMetersPerSecond)
      + pidController.calculate(getEncR1Rate(), speeds.frontRightMetersPerSecond));
    motorR2.setVoltage(simpleFeedforward.calculate(speeds.rearRightMetersPerSecond)
      + pidController.calculate(getEncR2Rate(), speeds.rearRightMetersPerSecond));*/

    /*motorL1.setVoltage(speeds.frontLeftMetersPerSecond);
    motorL2.setVoltage(speeds.rearLeftMetersPerSecond);
    motorR1.setVoltage(speeds.frontRightMetersPerSecond);
    motorR2.setVoltage(speeds.rearRightMetersPerSecond);*/

    motorL1.setVoltage(simpleFeedforward.calculate(speeds.frontLeftMetersPerSecond));
    motorL2.setVoltage(simpleFeedforward.calculate(speeds.rearLeftMetersPerSecond));
    motorR1.setVoltage(simpleFeedforward.calculate(speeds.frontRightMetersPerSecond));
    motorR2.setVoltage(simpleFeedforward.calculate(speeds.rearRightMetersPerSecond));

    //SmartDashboard.putNumber("motorL1", simpleFeedforward.calculate(speeds.frontLeftMetersPerSecond));
    //SmartDashboard.putNumber("motorR1", speeds.frontRightMetersPerSecond);

    //SmartDashboard.putNumber("angv", getAngV());
    //SmartDashboard.putNumber("vx", getVx());
    //SmartDashboard.putNumber("vy", getVy());
  }

  public void stopDrive(){ 
    /*motorL1.set(ControlMode.PercentOutput, 0);
    motorL2.set(ControlMode.PercentOutput, 0);
    motorR1.set(ControlMode.PercentOutput, 0);
    motorR2.set(ControlMode.PercentOutput, 0);*/

    motorL1.set(0);
    motorL2.set(0);
    motorR1.set(0);
    motorR2.set(0);
  }

  double maxMecSpeed = 9;
  public void setMotorsSimple(double xSpeed, double ySpeed, double rotSpeed) {
    /*motorL1.set(ControlMode.PercentOutput, (xSpeed - ySpeed - rotSpeed) / maxMecSpeed);
    motorL2.set(ControlMode.PercentOutput, (xSpeed + ySpeed - rotSpeed) / maxMecSpeed);
    motorR1.set(ControlMode.PercentOutput, (xSpeed + ySpeed + rotSpeed) / maxMecSpeed);
    motorR2.set(ControlMode.PercentOutput, (xSpeed - ySpeed + rotSpeed) / maxMecSpeed);*/

    motorL1.set((xSpeed - ySpeed - rotSpeed) / maxMecSpeed);
    motorL2.set((xSpeed + ySpeed - rotSpeed) / maxMecSpeed);
    motorR1.set((xSpeed + ySpeed + rotSpeed) / maxMecSpeed);
    motorR2.set((xSpeed - ySpeed + rotSpeed) / maxMecSpeed);

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

  public void updateOdometry() {
    poseEstimator.update(NavX.getGyroRotation2d(), getCurrentWheelSpeeds());
  }

   /** Updates the field relative position of the robot. */
   public void updateOdometryWithVision(double distance, double gyroAngle, double turretAngle, double tx) {
    //odometry.update(NavX.getGyroRotation2d(), getCurrentWheelSpeeds());
    poseEstimator.update(NavX.getGyroRotation2d(), getCurrentWheelSpeeds());

    poseEstimator.addVisionMeasurement(
      VisionPoseEstimation.getGlobalPoseEstimation(getPoseEstimate(), distance, gyroAngle, turretAngle, tx), 
      Timer.getFPGATimestamp() - Constants.limelightLatency);
  }

  public void resetEncoders() {
    encoderL1.reset();
    encoderL2.reset();
    encoderR1.reset();
    encoderR2.reset();
  }

  public void setPose(Pose2d pose) {
    resetEncoders();
    poseEstimator.resetPosition(pose, NavX.getGyroRotation2d());
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
  
  public double getEncL1Rate() {
    return encoderL1.getRate();
  }

  public double getEncL2Rate() {
    return encoderL2.getRate();
  }

  public double getEncR1Rate() {
    return encoderR1.getRate();
  }

  public double getEncR2Rate() {
    return encoderR2.getRate();
  }

  public MecanumDriveKinematics getMecKinematics() {
    return kinematics;
  }

  public SimpleMotorFeedforward getSimpleFeedforward() {
    return simpleFeedforward;
  }

  public Pose2d getPoseEstimate() {
    return poseEstimator.getEstimatedPosition();
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
    //double vz = curChassisSpeeds.vxMetersPerSecond; //velocity in the foward direction, relative to robot
    //double vx = -curChassisSpeeds.vyMetersPerSecond; //velocity towards the right, relative to robot
    double theta = Math.toRadians(90 - turretAngleDeg);
    //double output = vx * Math.cos(theta) + vz * Math.sin(theta);

    double output = -curChassisSpeeds.vyMetersPerSecond * Math.cos(theta) + curChassisSpeeds.vxMetersPerSecond * Math.sin(theta);

    SmartDashboard.putNumber("paraV", output);
    return output;
  }

  public double getPerpV(double turretAngleDeg) {
    //converts from robot-relative velocities directly to target relative velocities
    //double vz = curChassisSpeeds.vxMetersPerSecond; //velocity in the foward direction, relative to robot
    //double vx = -curChassisSpeeds.vyMetersPerSecond; //velocity towards the right, relative to robot
    double theta = Math.toRadians(90 - turretAngleDeg);

    //double output = -vx * Math.sin(theta) + vz * Math.cos(theta);
    double output = curChassisSpeeds.vyMetersPerSecond * Math.sin(theta) + curChassisSpeeds.vxMetersPerSecond * Math.cos(theta);

    SmartDashboard.putNumber("perpV", output);
    return output;
  }

  public double getAngV() {
    double output = Math.toDegrees(curChassisSpeeds.omegaRadiansPerSecond);
    //SmartDashboard.putNumber("angv", output);
    return output;
  }

  public double getVx() {
    //return -curChassisSpeeds.vyMetersPerSecond;
    return curChassisSpeeds.vxMetersPerSecond;
  }

  public double getVy() {
    //return curChassisSpeeds.vxMetersPerSecond;
    return curChassisSpeeds.vyMetersPerSecond;
  }
}
