// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int motorL1Value = 0;
    public static final int motorL2Value = 1;
    public static final int motorR1Value = 3;
    public static final int motorR2Value = 2;
    public static final int motorTurret = 90;
    public static final int motorShooterTop = 2;
    public static final int motorShooterBottom = 3;
    public static final int motorHood = 93;
    public static final int motorNeoTest = 1;

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

    //controller ports (where they plug in at)
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;

    //gta drive
    public static final double maxDriveSpeed = 1.0;

    //mecanum
    public static final int encL1A = 0;
    public static final int encL1B = 1;
    public static final int encL2A = 2;
    public static final int encL2B = 3;
    public static final int encR1A = 5;
    public static final int encR1B = 6;
    public static final int encR2A = 8;
    public static final int encR2B = 9;

    //mecanum trajectory
    public static final double maxMecSpeed = 5; //meters per second
    public static final double maxMecAcceleration = 5; //meters per second squared

    public static final double ksMecFeedForward = 0.5695; //determined from drive characterization
    public static final double kvMecFeedForward = 3.2686;
    public static final double kaMecFeedForward = 0.1794;

    public static final double kpMecXController = 0.1; //determined through testing
    public static final double kpMecYController = 0.1;
    public static final double kpMecThetaController = 0.1;

    private static final double maxMecRotationVelocity = 2; //what units??
    private static final double maxMecRotationAccel = 2; 
    public static final Constraints mecThetaControllerConstraints = 
        new Constraints(maxMecRotationVelocity, maxMecRotationAccel);
    
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
    public static final double limelightMountDegreeOffset = 0; 
    public static final double limelightLatency = 0.03;

    //pneumatics

    //turret
    public static final double turretSeekSpeed = 0.2;
    public static final double turretCircumference = 0;
    public static final int encTurretA = 90;
    public static final int encTurretB = 90;
    
    public static final double ksTurret = 0;
    public static final double kvTurret = 0;
    public static final double kaTurret = 0;
    public static final double kpTurret = 0;
    public static final Constraints turretConstraints = null;
    
    //hood
    public static final int encHoodA = 90;
    public static final int encHoodB = 90;
    
    public static final double ksHood = 0;
    public static final double kvHood = 0;
    public static final double kaHood = 0;
    public static final double kpHood = 0;
    public static final Constraints hoodConstraints = null;
	

    //shooter
    public static final double shootSpeed = 0.65;
    public static final double defaultShooterSpeed = 0.8;
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
    public static final double deadzone = 0.1;

    //field
    public static final double targetDeltaY = 2.64;
    
    //driver contoller buttons
    public static final int limelightTargetButton = rightBumper;
    public static final int LLDistanceButton = bButton;
    public static final int PneumForwardButton = xButton;
    public static final int PneumReverseButton = yButton;
    public static final int PneumToggleButton = rightBumper;
    public static final int zeroYawButton = yButton;
    public static final int shootButton = bButton;
    public static final int zeroEncButton = xButton;
    public static final int driveNormalButton = aButton;
    public static final int missTargetButton = 0;
    public static final double maxShooterV = 0;
    public static final double maxShooterA = 0;
    public static final double kpTopShooter = 0;
    public static final double kpBottomShooter = 0;
    
    
    
    
    
    
	
    
    
    
    
    
    
    


}
