// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//subsystem for the gyro on our robot
public class NavX extends SubsystemBase{
  private final static AHRS gyro = new AHRS(SPI.Port.kMXP);
  private static double gyroReverser = 1;
  double currAccelX, lastAccelX = 0, jerkX, accelY;
  boolean collisionDetected = false;
  
  /**
   * Creates a new NavX.
   */
  public NavX() { 
    //all methods are static so this constructor will never be called
    /*new Thread( () -> { 
      try {
        Thread.sleep(1000); 
      }
      catch(InterruptedException e) {
        SmartDashboard.putBoolean("interupted", true);
        return;
      }
      
      zeroGyroYaw(); 
    });*/
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currAccelX = getGyroAccelX();
    jerkX = currAccelX - lastAccelX;
    lastAccelX = currAccelX;
    //accelY = getGyroAccelY();

    if (jerkX > Constants.jerkXCollisionThreshold) {
      collisionDetected = true;
    }


    SmartDashboard.putNumber("gyroYaw", getGyroYaw());
    SmartDashboard.putNumber("gyroAngV", getGyroAngV());
    //SmartDashboard.putNumber("gyroAccelX", getGyroAccelX());
    //SmartDashboard.putNumber("gyroAccelY", getGyroAccelY());
    //SmartDashboard.putNumber("gyroJerkX", jerkX);
    //SmartDashboard.putBoolean("gyroIsCalibrating", gyroIsCalibrating());
    //SmartDashboard.putBoolean("CollisionDetected", collisionDetected);

  }

  public static double getGyroYaw() { //yaw is rotation (turning) left or right
    //negative because trajectory requires counterclockwise rotation to be positive
    return gyro.getYaw() * gyroReverser; 
  }

  //navx didnt have a built in getRotation2d method (in this WPILIB version) so i had to get it like this
  public static Rotation2d getGyroRotation2d() {
    //i made yaw negative because it needs to increase as it turns left to work for DiffDrive
    return Rotation2d.fromDegrees(getGyroYaw());
  }

  public static double getGyroAccelX(){
    return gyro.getWorldLinearAccelX();
  }

  public static double getGyroAccelY(){
    return gyro.getWorldLinearAccelY();
  }

  public static double getGyroAngV() {
    return gyro.getRate();
  }

  public static void zeroGyroYaw() {
    gyro.zeroYaw();
  }

  public static boolean gyroIsCalibrating() {
    return gyro.isCalibrating();
  }

  public static void reverseGyro() {
    gyroReverser = -1;
  }

  public static void unReverseGyro() {
    gyroReverser = 1;
  }

}