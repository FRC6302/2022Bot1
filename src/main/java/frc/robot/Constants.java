// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

/* 
  navx dio port to roborio ID mapping
  format: dio port -> rio ID 
  0 -> 10
  1 -> 11
  2 -> 12
  3 -> 13
  4 -> 18
  5 -> 19
  6 -> 20
  7 -> 21
  8 -> 22
  9 -> 23
  found at https://pdocs.kauailabs.com/navx-mxp/installation/io-expansion/
*/

public final class Constants {
  //controller values - wont ever change
  public static final int leftTrigger = 2;
  public static final int rightTrigger = 3;
  public static final int leftStickX = 0;
  public static final int leftStickY = 1;
  public static final int rightStickX = 4;
  public static final int rightStickY = 5;
  public static final int aButton = 1;
  public static final int bButton = 2;
  public static final int xButton = 3;
  public static final int yButton = 4;
  public static final int leftBumper = 5;
  public static final int rightBumper = 6;

  /* converting navx dio ports to roborio IDs - wont ever change
  found at https://pdocs.kauailabs.com/navx-mxp/installation/io-expansion/ */
  public static final int navxDIO0 = 10;
  public static final int navxDIO1 = 11;
  public static final int navxDIO2 = 12;
  public static final int navxDIO3 = 13;
  public static final int navxDIO4 = 18;
  public static final int navxDIO5 = 19;
  public static final int navxDIO6 = 20;
  public static final int navxDIO7 = 21;
  public static final int navxDIO8 = 22;
  public static final int navxDIO9 = 23;
  
  //motors
  public static final int motorL1Value = 58; //front left drive
  public static final int motorL2Value = 60;
  public static final int motorR1Value = 52;
  public static final int motorR2Value = 51;

  public static final int motorTurret = 11;
  public static final int motorShooterTop = 24;
  public static final int motorShooterBottom = 53; //spark 54 is broke!!!!
  public static final int motorHood = 55;
  public static final int motorNeoTest = 24; //new is 24
  public static final int motorFirstClimber = 59;
  public static final int motorSecondClimber = 57;
  public static final int motorIntake = 56;
  public static final int motorFeederFront = 12;
  public static final int motorFeederMiddle = 13;

  //encoders - DIO ports on rio or navx
  public static final int encL1A = 6;
  public static final int encL1B = 7;
  public static final int encL2A = navxDIO1;
  public static final int encL2B = navxDIO2;
  public static final int encR1A = navxDIO7;
  public static final int encR1B = navxDIO8;
  public static final int encR2A = 3;
  public static final int encR2B = 4;

  public static final int encTestA = navxDIO8;
  public static final int encTestB = navxDIO9;
  
  public static final int encTurretA = 0;
  public static final int encTurretB = 1;
  
  public static final int encHoodA = 90;
  public static final int encHoodB = 90;
  
  public static final int topShooterEncA = 7;
  public static final int topShooterEncB = 8;
  public static final int bottomShooterEncA = 0;
  public static final int bottomShooterEncB = 1;
  
  //controller ports (where they plug in at) - laptop USB
  public static final int driverControllerPort = 0;
  public static final int operatorControllerPort = 1;

  //gta drive
  public static final double maxDriveSpeed = 1.0;

  //mecanum
  public static final double maxMecAttainableWheelSpeed = 5; //a guess
  public static final int vxFilterSize = 10;
  public static final int vyFilterSize = 10;

  //mecanum trajectory
  public static final double maxMecSpeed = 5; //meters per second
  public static final double maxMecAcceleration = 5; //meters per second squared

  public static final double ksMecFeedForward = 0.128; //gym floor
  public static final double kvMecFeedForward = 2.7664;
  public static final double kaMecFeedForward = 0.41056;

  //tune these by graphing desired position and actual pos on glass
  public static final double kpMecPosXController = 2; //determined through testing
  public static final double kpMecPosYController = 2;
  public static final double kpMecThetaController = 2;

  public static final double maxMecRotationVelocity = 7; // rad/s
  public static final double maxMecRotationAccel = 10; // rad/s/s
  
  public static final double kpMecL1Velocity = 1; //i think these should all be similar
  public static final double kpMecL2Velocity = 1;
  public static final double kpMecR1Velocity = 1;
  public static final double kpMecR2Velocity = 1;
  
  public static final Constraints mecVConstraints = null;
  public static final double kpMecV = 0;
  
  //gyro
  public static final double jerkXCollisionThreshold = 1;
  public static final int gyroVelocityFilterSamples = 5;

  //limelight
  public static final double limelightLatency = 0.025; //20 ms

  //limelight goal
  public static final double limelightGoalMountDegreeOffset = 32;//22;// 36.5; //the angle the limelight is pointing relative to the ground, 0 = parallel to floor

  //limelight balls
  
  public static final double limelightBallMountDegreeOffset = 0;


  //pneumatics


  //turret
  public static final double turretSeekSpeed = 0.5;
  //public static final double turretCircumference = 0;
  public static final double minTurretAngle = -110; //-230
  public static final double maxTurretAngle = 280; //200
  public static final double turretResetVoltage = 8;

  /*public static final double ksTurret = 0.56996;
  public static final double kvTurret = 0.0161;
  public static final double kaTurret = 0.0019;*/
  public static final double ksTurret = 0.4;//.5 //.64
  public static final double kvTurret = 0.0167;
  public static final double kaTurret = 0.0019;
  public static final double kpPosTurret = 7; //5 //sysid guessed 35
  public static final double kdPosTurret = 0; //sysid guessed 2
  public static final double kpVelTurret = 0.05; //sysid said 

  public static final double maxTurretV = .01; //deg/s
  public static final double maxTurretA = .01; //deg/s/s
  
  
  
  //hood
  public static final double ksHood = 0.10; //0.17; //from sysid
  public static final double kvHood = 28.8;
  public static final double kaHood = 0.782;
  public static final double kgHood = 0.1; //voltage needed to overcome gravity at horizontal. SYSId calcs it at enc=0?? That's problem

  public static final double kpHood = 3.5; //4
  public static final double kdHood = 0; //2

  public static final double maxHoodV = Units.degreesToRadians(10); // deg/s to rad/s
  public static final double maxHoodA = Units.degreesToRadians(20); // deg/s/s to rad/s/s
  
  public static final double hoodMinimumAngle = Units.degreesToRadians(20);
  
  

  //shooter
  public static final double defaultShooterSpeed = 0.8;
  public static final double topShooterDefaultVolts = 2;
  public static final double bottomShooterDefaultVolts = 2;

  public static final double maxShooterV = 15; // m/s
  public static final double maxShooterA = 10; // m/s/s
  
  public static final double kpTopShooter = 0.005; //sysid 1
  public static final double ksTopShooter = 0.012;
  public static final double kVTopShooter = 0.020957; //tested top, copying to bottom for now
  public static final double kATopShooter = 0.001948;
  
  public static final double kpBottomShooter = 0.0001;
  public static final double ksBottomShooter = 0.001;
  public static final double kVBottomShooter = 0.02148;
  public static final double kABottomShooter = 0.00273;
  
  public static final int velocityPeriodsToAverage = 7;

  
  
  //intake
  public static final double intakeDefaultVolts = 5;

  //feeder
  public static final double feederFrontDefaultVolts = 3;
  public static final double feederMiddleDefaultVolts = 4;


  

  //climber
  public static final double maxClimbV = 0.5; // m/s?
  public static final double maxClimbA = 0.5;
  public static final double kpPosClimb = 0.1;
  
  public static final double ksClimb = 0; //from sysid
  public static final double kvClimb = 0;
  public static final double kaClimb = 0;


  //move command
  public static final double leftMotorsMoveSpeed = 0.1;
  public static final double rightMotorsMoveSpeed = 0.1;
  public static final double MoveTime = 0.5; 

  //ball tracking
  public static final double timeBetweenTrajectoryGeneration = 0.2; //seconds
  public static final double ballTrackingDistanceTolerance = 1.5; //meters
  public static final double ballTrackingDegTolerance = 15; //degrees


  //miscellaneous
  public static final double turningRate = 0.5;
  public static final double deadzone = 0.3;
  public static final double loopTime = 0.020;
  public static final double visionPoseDeltaTolerance = 5; //meters
  public static final int sparkCANTimeoutMs = 50;

  //missing the goal on purpose
  public static final double turretOffsetForMissing = 50;
  
  public static final double topShooterVoltsForMissing = 2; //make top volts negative? Ultra backspin
  public static final double bottomShooterVoltsForMissing = 2;

  public static final double hoodAngleForMissingTarget = hoodMinimumAngle +  Units.degreesToRadians(5);
  
  public static final double waitToFeedOppositeBallTime = 2; //seconds
  public static final double timeToShootOppositeBall = 1;


  //field
  //public static final double targetHeightInches = 104;
  public static final Translation2d goalLocation = new Translation2d(Units.feetToMeters(27), Units.feetToMeters(13.5));
  public static final double goalOutsideRadius = Units.feetToMeters(2); //0.67786;

  //robot construction
  public static final double limelightToRobotCenterRadius = Units.inchesToMeters(9.5);
  public static final double limelightToTargetHeight = Units.inchesToMeters(62.5 - 25.0625); //Units.inchesToMeters(103 - 25.0625);
  public static final double limelightToBallCenterHeight = Units.inchesToMeters(40 - 4.25); //find out what ball height to use
  public static final double robotWheelToWheelWidth = Units.inchesToMeters(22.5);
  public static final double robotWheelToWheelLength = Units.inchesToMeters(20 + 5./16.);
  
  //driver contoller buttons
  public static final int driveButton = leftBumper;
  public static final int trackButton = bButton; 
  public static final int zeroYawButton = yButton; //

//operator buttons
 
  public static final int shootButton = xButton; 
  public static final int zeroEncButton = yButton;
  public static final int driveNormalButton = aButton;
  public static final int missTargetButton = yButton;
  public static final int shootButton2 = bButton;
  public static final int turnTurretButton = rightBumper; 
  public static final int zeroTurretButton = xButton; //
  public static final int raiseHoodButton = yButton; 
  public static final int raiseHood2Button = leftBumper; 
  public static final int feedBothButton = aButton;
  public static final int intakeButton = aButton;
  public static final int closeToBallsButton = aButton; //
  public static final int farAwayButton = bButton; //
  public static final int turnTurret2Button = yButton;
  public static final int zeroHoodButton = yButton; //
  public static final int moveClimbersButton = rightBumper;
  public static final int reverseButton = bButton; 





  







  
  






}
