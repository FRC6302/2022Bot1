// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.Util;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.library.Utilities;

//subsystem for the gyro on our robot
public class NavX extends SubsystemBase{
  private final static AHRS gyro = new AHRS(SPI.Port.kMXP);
  private static double gyroReverser = 1;
  //double currAccelX, lastAccelX = 0, jerkX, accelY;
  //boolean collisionDetected = false;

  private  static double offsetAngle = 0;

  private static MedianFilter vxFilter = new MedianFilter(Constants.gyroVelocityFilterSamples);
  
  /**
   * Creates a new NavX.
   */
  public NavX() { 
    /*new Thread(() -> { 
      try {
        Thread.sleep(1000); 
      }
      catch(InterruptedException e) {
        SmartDashboard.putBoolean("interupted", true);
        return;
      }
      
      zeroGyroYaw(); 
    });*/
    offsetAngle = 0;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*currAccelX = getGyroAccelX();
    jerkX = currAccelX - lastAccelX;
    lastAccelX = currAccelX;
    //accelY = getGyroAccelY();

    if (jerkX > Constants.jerkXCollisionThreshold) {
      collisionDetected = true;
    }*/


    SmartDashboard.putNumber("gyroYaw", getGyroYaw());
    SmartDashboard.putNumber("gyroAngV", getGyroAngV());
    SmartDashboard.putNumber("gyro vx", getGyroGlobalVx());
    SmartDashboard.putNumber("gyro vy", getGyroGlobalVy());
    SmartDashboard.putNumber("gyro accum ang", getAccumulatedAngle());
    //SmartDashboard.putNumber("gyroAccelX", getGyroAccelX());
    //SmartDashboard.putNumber("gyroAccelY", getGyroAccelY());
    //SmartDashboard.putNumber("gyroJerkX", jerkX);
    //SmartDashboard.putBoolean("gyroIsCalibrating", gyroIsCalibrating());
    //SmartDashboard.putBoolean("CollisionDetected", collisionDetected);

  }

  public static double getGyroYaw() { //yaw is rotation (turning) left or right
    //negative because trajectory requires counterclockwise rotation to be positive
    return Utilities.constrainAngle(getAccumulatedAngle()); 
  }

  public static double getAccumulatedAngle() {
    return (-1.034818 * gyro.getAngle()) + offsetAngle; //pretty sure it is plus
    //gyro.setAngleAdjustment(adjustment);
  }

  //ccw+
  public static void setAngleOffset(double offsetDeg) {
    offsetAngle = offsetDeg;
  }

  //navx didnt have a built in getRotation2d method (in this WPILIB version) so i had to get it like this
  public static Rotation2d getGyroRotation2d() {
    //i made yaw negative because it needs to increase as it turns left to work for DiffDrive
    return Rotation2d.fromDegrees(getGyroYaw());
  }

  public static double getGyroGlobalVx() {
    return vxFilter.calculate(-gyro.getVelocityX());
  }

  public static double getGyroGlobalVy() {
    return gyro.getVelocityY();
  }

  public static double getGyroAccelX(){
    return gyro.getWorldLinearAccelX();
  }

  public static double getGyroAccelY(){
    return gyro.getWorldLinearAccelY();
  }

  public static double getGyroAngV() {
    //return Units.radiansToDegrees(-gyro.getRate());
    return -gyro.getRate() * gyro.getActualUpdateRate();
  }

  public static void zeroGyroYaw() {
    gyro.zeroYaw();
    //gyro.setAngleAdjustment(adjustment);
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