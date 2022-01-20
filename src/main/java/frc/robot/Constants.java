// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int motorL1Value = 97;
    public static final int motorL2Value = 1;
    public static final int motorR1Value = 2;
    public static final int motorR2Value = 3;
    public static final int motorTurret = 90;

    //controller values
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

    //drive
    public static final double maxDriveSpeed = 1.0;

    //mecanum
	public static final double maxMecSpeed = 0;

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

    //pneumatics

    //turret
    public static final double turretSeekSpeed = 0;


    //move command
    public static final double leftMotorsMoveSpeed = 0.1;
    public static final double rightMotorsMoveSpeed = 0.1;
    public static final double MoveTime = 0.5;

    //miscellaneous
    public static final double turningRate = 0.5;
    public static final double deadzone = 0.1;

    //driver contoller buttons
    public static final int limelightTargetButton = Constants.aButton;
    public static final int LLDistanceButton = bButton;
    public static final int PneumForwardButton = xButton;
    public static final int PneumReverseButton = yButton;
    public static final int PneumToggleButton = rightBumper;
    
    
    
    
    


}
