// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

import edu.wpi.first.math.util.Units;

/** class for storing all of data that we got from testing our shooter/hood */
public class Data {

  private static double[][] distanceAngleData = { 
    {1, Units.degreesToRadians(10)}, //{distance in meters, angle in degrees} format
    {3, Units.degreesToRadians(15)}, 
    {5, Units.degreesToRadians(20)},
    {7, Units.degreesToRadians(25)},
    {9, Units.degreesToRadians(30)},
    {11, Units.degreesToRadians(35)},
    {13, Units.degreesToRadians(40)}
  };

  private static double[][] distanceTimeData = { 
    {1.0, 2}, //{distance in meters, time in sec} format
    {3.0, 3}, 
    {10, 5} 
  };

  private static double[][] distanceTopShooterData = { 
    {1.0, 2}, //{distance in meters, motor voltage} format
    {3.0, 3}, 
    {10, 5} 
  };

  private static double[][] distanceBottomShooterData = { 
    {1.0, 2}, //{distance in meters, motor voltage} format
    {3.0, 3}, 
    {10, 5} 
  };


  public static LinearInterpolator distanceAngleMap;
  public static LinearInterpolator distanceTimeMap;
  public static LinearInterpolator distanceTopShooterMap;
  public static LinearInterpolator distanceBottomShooterMap;

  
  public Data() {
    distanceAngleMap = new LinearInterpolator(distanceAngleData);
    distanceTimeMap = new LinearInterpolator(distanceTimeData);
    distanceTopShooterMap = new LinearInterpolator(distanceTopShooterData);
    distanceBottomShooterMap = new LinearInterpolator(distanceBottomShooterData);
  }
  

  public static double getHoodAngle(double distance) {
    return distanceAngleMap.getInterpolatedValue(distance);
  }

  public static double getAirtime(double distance) {
    return distanceTimeMap.getInterpolatedValue(distance);
  }

  public static double getTopShooterVoltage(double distance) {
    return distanceTopShooterMap.getInterpolatedValue(distance);
  }

  public static double getBottomShooterVoltage(double distance) {
    return distanceBottomShooterMap.getInterpolatedValue(distance);
  }
}
