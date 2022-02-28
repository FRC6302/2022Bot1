// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //motors
    public static final int motorL1Value = 1;
    public static final int motorL2Value = 2;
    public static final int motorR1Value = 4;
    public static final int motorR2Value = 3;
    public static final int motorTurret = 0;
    public static final int motorShooterTop = 3;
    public static final int motorShooterBottom = 2;
    public static final int motorHood = 93;
    public static final int motorNeoTest = 1;

    //encoders - DIO ports on rio
    public static final int encL1A = 0;
    public static final int encL1B = 1;
    public static final int encL2A = 2;
    public static final int encL2B = 3;
    public static final int encR1A = 6;
    public static final int encR1B = 7;
    public static final int encR2A = 8;
    public static final int encR2B = 9;
    
    public static final int encTurretA = 4;
    public static final int encTurretB = 5;
    
    public static final int encHoodA = 90;
    public static final int encHoodB = 90;
    
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

    //controller ports (where they plug in at) - laptop USB
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;

    //gta drive
    public static final double maxDriveSpeed = 1.0;

    //mecanum
    

    //mecanum trajectory
    public static final double maxMecSpeed = 5; //meters per second
    public static final double maxMecAcceleration = 5; //meters per second squared

    public static final double ksMecFeedForward = 0.5695; //determined from drive characterization
    public static final double kvMecFeedForward = 3.2686;
    public static final double kaMecFeedForward = 0.1794;

    public static final double kpMecXController = 0.1; //determined through testing
    public static final double kpMecYController = 0.1;
    public static final double kpMecThetaController = 0.1;

    public static final double maxMecRotationVelocity = 5; // rad/s
    public static final double maxMecRotationAccel = 5; // rad/s/s
    
    public static final double kpMecL1Velocity = 0.1; //i think these should all be similar
    public static final double kpMecL2Velocity = 0.1;
    public static final double kpMecR1Velocity = 0.1;
    public static final double kpMecR2Velocity = 0.1;
    
    public static final Constraints mecVConstraints = null;
    public static final double kpMecV = 0;
    
    //gyro
    public static final double jerkXCollisionThreshold = 1;

    //limelight
    public static final double limelightTXDeadzone = 0.5;
    public static final double limelightTargetXScaling = 1;
    public static final double limelightSpecificArea = 27;
    public static final double limelightTargetDistanceSpeed = 0.3;
    public static final double limelightSeekSpeed = 0.3;
    public static final double limelightTargetXSpeed = 0.2;
    public static final double limelightGetInRangeSpeed = 0.2;
    public static final double limelightTargetArea = 0.3;
    //the angle the limelight is pointing relative to the ground, 0 = parallel to floor
    public static final double limelightMountDegreeOffset = 21; 
    public static final double limelightLatency = 0.02; //20 ms

    //pneumatics


    //turret
    public static final double turretSeekSpeed = 0.5;
    //public static final double turretCircumference = 0;
    public static final double minTurretAngle = -180;
    public static final double maxTurretAngle = 180;

    public static final double ksTurret = 0.56996;
    public static final double kvTurret = 0.0161;
    public static final double kaTurret = 0.0019;

    //public static final double kpTurret = 0.05; 
    public static final double kpPosTurret = 1.5; //sysid guessed 2
    public static final double kpVelTurret = 0.05;

    public static final double maxTurretV = 180; //deg/s
    public static final double maxTurretA = 90; //deg/s/s
    
    
    
    //hood

    public static final double ksHood = 0;
    public static final double kvHood = 0;
    public static final double kaHood = 0;
    public static final double kpHood = 0;
    public static final Constraints hoodConstraints = null;
	

    //shooter
    public static final double shootSpeed = 0.65;
    public static final double defaultShooterSpeed = 0.8;
    public static final double maxShooterV = 0;
    public static final double maxShooterA = 0;
    public static final double kpTopShooter = 0;
    public static final double kpBottomShooter = 0;
    public static final int topShooterEncA = 7;
    public static final int topShooterEncB = 8;
    public static final int bottomShooterEncA = 0;
    public static final int bottomShooterEncB = 1;

    //move command
    public static final double leftMotorsMoveSpeed = 0.1;
    public static final double rightMotorsMoveSpeed = 0.1;
    public static final double MoveTime = 0.5; 

    //miscellaneous
    public static final double turningRate = 0.5;
    public static final double deadzone = 0.3;

    //field
    public static final double targetDeltaY = 0.724;
    public static final Translation2d goalLocation = new Translation2d(0, 0);
    public static final double goalOutsideRadius = 0.67786;
    public static final double limelightToRobotCenterDistance = -0.2;
    
    //driver contoller buttons
    public static final int limelightTargetButton = rightBumper;
    public static final int LLDistanceButton = bButton;
    public static final int PneumForwardButton = xButton;
    public static final int PneumReverseButton = yButton;
    public static final int PneumToggleButton = rightBumper;
    public static final int zeroYawButton = leftBumper;
    public static final int shootButton = aButton;
    public static final int zeroEncButton = yButton;
    public static final int driveNormalButton = aButton;
    public static final int missTargetButton = 0;
    public static final int shootButton2 = bButton;
    public static final int turnTurretButton = bButton;
    public static final int zeroTurretButton = xButton;
    
    
    
    
    
    
    
    
    
	
    
    
    
    
    
    
    


}
