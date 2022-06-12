// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

import edu.wpi.first.math.util.Units;

/** class for storing all of data that we got from testing our shooter/hood */
public class Data {

  //comp
  /*private static double[][] distanceAngleData = { 
    {1.981, Units.degreesToRadians(20)}, //{distance in meters, angle in degrees} format
    {3.048, Units.degreesToRadians(26)}, 
    {4.572, Units.degreesToRadians(32)},
    {6.096, Units.degreesToRadians(38)},
    {7.62, Units.degreesToRadians(44)},
    {9.144, Units.degreesToRadians(46)},
    {10.668, Units.degreesToRadians(48)}
  };*/

  //8th grade night
  private static double[][] distanceAngleData = { 
    {1.981, Units.degreesToRadians(20)}, //{distance in meters, angle in degrees} format
    {3.048, Units.degreesToRadians(30)}, 
    {4.572, Units.degreesToRadians(40)},
    {6.096, Units.degreesToRadians(40)},
    {7.62, Units.degreesToRadians(40)},
    {9.144, Units.degreesToRadians(40)},
    {10.668, Units.degreesToRadians(40)}
  };

  private static double[][] distanceTimeData = { 
    {1.981, 0.8}, //{distance in meters, time in sec} format
    {3.048, 0.85}, 
    {4.572, 0.93},
    {6.096, 1.03},
    {7.62, 1.15},
    {9.144, 1.36},
    {10.668, 1.6}
  };

  private static double[][] distanceTopShooterData = { 
    {1.981, 225}, //{distance in meters, motor vel rad/s} format
    {3.048, 287.5}, 
    {4.572, 325},
    {6.096, 400},
    {7.62, 490},
    {9.144, 550},
    {10.668, 550}
  };

  private static double[][] distanceBottomShooterData = { 
    {1.981, 65}, //good //{distance in meters, motor vel rad/s} format
    {3.048, 75}, 
    {4.572, 104},
    {6.096, 140},
    {7.62, 190},
    {9.144, 245},
    {10.668, 300}
  };

  /* from practice field, not good
  private static double[][] distanceBottomShooterData = { 
    {1.981, 65}, //good //{distance in meters, motor vel rad/s} format
    {3.048, 75}, 
    {4.572, 115},
    {6.096, 139},
    {7.62, 180},
    {9.144, 255},
    {10.668, 320}
  };*/

  //worked before comp
  /*private static double[][] distanceBottomShooterData = { 
    {1.981, 65}, //good //{distance in meters, motor vel rad/s} format
    {3.048, 71}, 
    {4.572, 84},
    {6.096, 99},
    {7.62, 125},
    {9.144, 185},
    {10.668, 230}
  };*/

  //data from when color guard was in gym
  /*private static double[][] distanceBottomShooterData = { 
    {1.981, 80}, //good //{distance in meters, motor vel rad/s} format
    {3.048, 100}, 
    {4.572, 160},
    {6.096, 210},
    {7.62, 245},
    {9.144, 270},
    {10.668, 315}
  };*/


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

  public static double getTopShooterVel(double distance) {
    return distanceTopShooterMap.getInterpolatedValue(distance);
  }

  public static double getBottomShooterVel(double distance) {
    return distanceBottomShooterMap.getInterpolatedValue(distance);
  }
}
