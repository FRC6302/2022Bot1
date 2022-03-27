// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

import edu.wpi.first.math.util.Units;

/** class for storing all of data that we got from testing our shooter/hood */
public class Data {

  private static double[][] distanceAngleData = { 
    {1.981, Units.degreesToRadians(20)}, //{distance in meters, angle in degrees} format
    {3.048, Units.degreesToRadians(28)}, 
    {4.572, Units.degreesToRadians(35)},
    {6.096, Units.degreesToRadians(37)},
    {7.62, Units.degreesToRadians(40)},
    {9.144, Units.degreesToRadians(45)},
    {10.668, Units.degreesToRadians(46)}
  };

  private static double[][] distanceTimeData = { 
    {1.981, 2}, //{distance in meters, time in sec} format
    {3.048, 2}, 
    {4.572, 2},
    {6.096, 2},
    {7.62, 3},
    {9.144, 3},
    {10.668, 3}
  };

  private static double[][] distanceTopShooterData = { 
    {1.981, 170}, //{distance in meters, motor vel rad/s} format
    {3.048, 200}, 
    {4.572, 265},
    {6.096, 390},
    {7.62, 500},
    {9.144, 550},
    {10.668, 550}
  };

  private static double[][] distanceBottomShooterData = { 
    {1.981, 95}, //{distance in meters, motor vel rad/s} format
    {3.048, 90}, 
    {4.572, 85},
    {6.096, 140},
    {7.62, 195},
    {9.144, 240},
    {10.668, 315}
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
