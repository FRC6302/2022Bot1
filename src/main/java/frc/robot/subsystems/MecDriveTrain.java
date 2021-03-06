// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.library.MecanumDrivePoseEstimator;
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

  //PIDController pidController = new PIDController(Constants.kpMecV, 0, 0);

  //robot width wheel-to-wheel is 0.584 m, length wheel-to-wheel is 0.521 m
  //0.584/2 = 0.292, 0.521/2 = 0.2605
  //use (y, -x) if using normal math graphs, the origin is center of robot
  /*private final Translation2d frontLeftLocation = new Translation2d(Constants.robotWheelToWheelLength/2, Constants.robotWheelToWheelWidth/2);
  private final Translation2d frontRightLocation = new Translation2d(Constants.robotWheelToWheelLength/2, -Constants.robotWheelToWheelWidth/2);
  private final Translation2d backLeftLocation = new Translation2d(-Constants.robotWheelToWheelLength/2, Constants.robotWheelToWheelWidth/2);
  private final Translation2d backRightLocation = new Translation2d(-Constants.robotWheelToWheelLength/2, -Constants.robotWheelToWheelWidth/2);

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
  

  private ChassisSpeeds curChassisSpeeds = new ChassisSpeeds();

  private Field2d field = new Field2d();*/

  //trying to figure how to get kalman filter for robot vx, don't know what i'm doing
  /*private final UnscentedKalmanFilter<N1, N1, N1> vxFilter = new UnscentedKalmanFilter<>(
    Nat.N1(), 
    Nat.N1(), 
    (x, u) -> u,
    (x, u) -> x.extractRowVector(2),
    VecBuilder.fill(5), 
    VecBuilder.fill(5), 
    Constants.loopTime);
  */
    
  private final double distancePerPulse = (Math.PI * 0.1524 / 2048) / 1; //mec wheel diam is 6 in or 0.1524 m

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
    //encTest = motorL2.getAlternateEncoder(2048);
    //motorL1.getEncoder(encoderType, countsPerRev)
    //encTest.setPositionConversionFactor(1);
    //encTest.setVelocityConversionFactor(1);
    
    //don't remember why i put this
    motorL1.setCANTimeout(Constants.sparkCANTimeoutMs);
    motorL2.setCANTimeout(Constants.sparkCANTimeoutMs);
    motorR1.setCANTimeout(Constants.sparkCANTimeoutMs);
    motorR2.setCANTimeout(Constants.sparkCANTimeoutMs);


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

    //how accurate the vision pose estimation is. The error comes from gyro and turret encoder drift and limelight targeting the wrong thing
    //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.001));
    
    //mecDrive = new MecanumDrive(motorL1, motorL2, motorR1, motorR2);

    //use this to show balls on the field?
    //field.getObject("red balls").setPose(new Pose2d());
    

    //SmartDashboard.putData("field", field);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("enc L1 V", getEncL1Rate());
    SmartDashboard.putNumber("enc L2 V", getEncL2Rate());
    SmartDashboard.putNumber("enc R1 V", getEncR1Rate());
    SmartDashboard.putNumber("enc R2 V", getEncR2Rate());

    //SmartDashboard.putNumber("test enc pos", encTest.getPosition());
    //SmartDashboard.putNumber("test enc vel", encTest.getVelocity());

    //updates the drive odometry every loop. Not updating using vision here because we prob wont be able to see the goal the whole match
    //updateOdometry();

    //SmartDashboard.putNumber("pose x", getPoseEstimate().getX());
    //SmartDashboard.putNumber("pose y", getPoseEstimate().getY());
    
    //field.setRobotPose(getPoseEstimate());

    //SmartDashboard.putNumber("mecanum vx", getVx());
    //SmartDashboard.putNumber("mecanum vy", getVy());
    //SmartDashboard.putNumber("mecanum ang v", getAngV());

    //used for converting robot velocity into target-relative velocity
    //curChassisSpeeds = kinematics.toChassisSpeeds(getCurrentWheelSpeeds());
    
  }
  
  

  /*public static MecanumDrive.WheelSpeeds driveCartesianIK(double ySpeed, 
  double xSpeed, double zRotation, double gyroAngle) {
    return mecDrive.driveCartesianIK(ySpeed, xSpeed, zRotation, gyroAngle);
  }*/

  //this method combines setMotorsFieldRel() and setMotorsRobotRel(), it's ugly tho
  public void setMotors(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds =
      RobotState.getMecKinematics().toWheelSpeeds(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, NavX.getGyroRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot));

    mecanumDriveWheelSpeeds.desaturate(Constants.maxMecAttainableWheelSpeed);

    setSpeeds(mecanumDriveWheelSpeeds); 
  }

  public void setMotorsFieldRel(double xSpeed, double ySpeed, double rot) {
    /* you create a ChassisSpeeds object from the field-relative speed arguments, and convert it into a MecanumDriveWheelSpeeds.
    The MecanumDriveKinematics to do this is located in RobotState so you have to call it from there */
    MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds =
      RobotState.getMecKinematics().toWheelSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, NavX.getGyroRotation2d()));

    mecanumDriveWheelSpeeds.desaturate(Constants.maxMecAttainableWheelSpeed);

    setSpeeds(mecanumDriveWheelSpeeds); 
  }

  public void setMotorsRobotRel(double xSpeed, double ySpeed, double rot) {
    /* you create a ChassisSpeeds object from the robot-relative speed arguments, and convert it into a MecanumDriveWheelSpeeds.
    The MecanumDriveKinematics to do this is located in RobotState so you have to call it from there */
    MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds =
      RobotState.getMecKinematics().toWheelSpeeds(
        new ChassisSpeeds(xSpeed, ySpeed, rot));

    mecanumDriveWheelSpeeds.desaturate(Constants.maxMecAttainableWheelSpeed);

    setSpeeds(mecanumDriveWheelSpeeds); 
  }

  /*takes in the MecanumDriveWheelSpeeds (generated in other methods) and converts it into actual motor commands. Having a 
    method to do this saves me repeating the same code in the various driving methods*/
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
  //setting the motors using the simple mecanum formulas. Pass in desired robot-relative speeds and get motor commands
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

  public SimpleMotorFeedforward getSimpleFeedforward() {
    return simpleFeedforward;
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

  
}
