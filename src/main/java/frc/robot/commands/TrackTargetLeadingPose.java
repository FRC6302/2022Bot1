// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightGoal;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/* tracks target like TrackTargetStationary command but accounts for the robots movement by leading the shot
so that you can shoot while moving*/
public class TrackTargetLeadingPose extends CommandBase {
  MecDriveTrain mecDriveTrain;
  Turret turret;
  Hood hood;
  Shooter shooter;

  //outputs of command
  //double desiredTurretV = 0, desiredTurretAngle = 0;
  //double hoodAdjust = 0, desiredHoodAngle = 0;

  //from chassis
  double paraV = 0, perpV = 0, angV = 0, vx = 0, vy = 0;
  
  //from limelight
  double rawDistance = 3, x = 0, y = 0, lastX = 0, lastY = 0;

  double gyroYaw = 0, ax = 0, ay = 0;

  double turretAngle = 0;
  //boolean isAllianceBall = true;

  //leading stuff
  double offsetAngle = 0; //radians
  double effectiveDistance = 3; //meters
  double actualDistance = effectiveDistance;
  double airTime = 3; //seconds
  double temp = 0;

  Translation2d effectiveGoal = new Translation2d();

  //approximating derivative stuff
  double prevOffset = offsetAngle, offsetDerivative = 0;
  double prevAirtime = airTime, airTimeDerivative = 0;
  double effectiveDistanceDerivative = 0, effectiveDistancePrediction = effectiveDistance;

  //pose stuff
  private Pose2d robotPose;
  double poseX = 0, poseY = 0;
  //private Transform2d robotToGoal = new Transform2d();
  //private Translation2d robotLocation = new Translation2d();
  //double estimatedActualDistance = 0;

  double angleToTarget = 0;


  /** Creates a new TrackTargetLeading. */
  public TrackTargetLeadingPose(MecDriveTrain mecDriveTrain, Turret turret, Hood hood, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;

    //not adding drivetrain to requirements bc that would not let us drive
    addRequirements(turret, hood, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose = RobotState.getPoseEstimate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*gyroYaw = NavX.getGyroYaw();
    turretAngle = turret.getAngle();

    if (LimelightGoal.getTargetFound()) { //runs when the LL can see the target
      rawDistance = LimelightGoal.getTargetDistance();
      x = LimelightGoal.getX();
      
      RobotState.updateOdometryWithVision(rawDistance, gyroYaw, turretAngle, x);
    }

    /*robotPose = RobotState.getPoseEstimate();
    poseX = robotPose.getX();
    poseY = robotPose.getY();

    actualDistance = robotPose.getTranslation().getDistance(Constants.goalLocation);
    //robotToGoal = new Transform2d(robotPose, goalPose);
    //distance = Math.sqrt(Math.pow(robotToGoal.getX(), 2) + Math.pow(robotToGoal.getY(), 2)); 

    //velocities with respect to target
    paraV = RobotState.getParaV(turretAngle);
    perpV = RobotState.getPerpV(turretAngle);

    vx = RobotState.getVx();
    vy = RobotState.getVy();
    
    angV = NavX.getGyroAngV();

    ax = NavX.getGyroAccelX();
    ay = NavX.getGyroAccelY();

    //guess airtime 0.020 sec into future? linear approximation?
    // airtime = airtime + 0.020 * derivative

    //dont wanna use pose for shooting bc if thats wrong then we'll miss. But we will miss anyways?
    effectiveGoal = new Translation2d(poseX + vx * airTime, poseY + vy * airTime);

    //temp value storing a calculation so that it doesnt have recalc it every time i need the value
    temp = airTime * (vy * (poseX - Constants.goalLocation.getX()) 
      + vx * (poseY - Constants.goalLocation.getY()));

    //temp = airTime * (vy * Math.sin(gyroYaw + turretAngle - x) + vx * Math.cos(gyroYaw + turretAngle - x));
    //TODO: compare these temp values and make sure they are close

    //this effectiveDistance calculation uses the offset angle prediction from previous loop
    //effectiveDistance = temp / (actualDistance * Math.sin(offsetAngle));
    //effectiveDistance = 
    //once we know the effective distance, then we can determine what the actual offset angle should be
    offsetAngle = Math.asin(temp / effectiveDistance);

    //take derivate of effective distance formula to get distance adjustment approximation to plug into airtime. a = 0?

    //predictedDistance = distance / Math.cos(Math.toRadians(offsetAngle)); //something like this

    //desiredHoodAngle = -3 * distance + 85; //degrees

    hood.setMotorPosPID(effectiveDistance, paraV);

    //turret.setMotorPosPID(angleToTarget + Units.radiansToDegrees(offsetAngle) - gyroYaw, perpV, effectiveDistance, angV); 

    //shooter.shootWithInitialBallVelocity(paraV, perpV, desiredHoodAngle, desiredTurretAngle, distance);
    //shooter.setMotorsVelPID(predictedDistance);

    /*prevOffset = offsetAngle;
    prevAirtime = airTime;

    //effectiveDistance += 0.020 * derivative gives the effective distance in 0.020 seconds.
    //then you can get the air time and offset angle for the next loop
    airTime = Data.getAirtime(effectiveDistance);

    //these derivatives are very approximate, find better way to do them?
    offsetDerivative = (offsetAngle - prevOffset) / Constants.loopTime;
    airTimeDerivative = (airTime - prevAirtime) / Constants.loopTime;

    //found the formula for this by differentiating by hand
    effectiveDistanceDerivative = airTimeDerivative * effectiveDistance / airTime
      + airTime *  ((poseX * ay - poseY * ax) / (actualDistance * Math.sin(offsetAngle)) 
      - (poseX * vy - poseY * vx) * (Math.sin(offsetAngle) * (poseX * vx + poseY * vy) / actualDistance + actualDistance * Math.cos(offsetAngle) * offsetDerivative)) 
      / Math.pow(actualDistance * Math.sin(offsetAngle), 2);

    //predicts what the effective distance will be in 20 ms when the loop runs again
    effectiveDistancePrediction = effectiveDistance + Constants.loopTime * effectiveDistanceDerivative;

    //predicts airtime for the next loop based on the effective distance prediction
    airTime = Data.getAirtime(effectiveDistancePrediction);
    //predicts the next offset angle 
    offsetAngle = Math.asin((airTime * temp / prevAirtime) / effectiveDistancePrediction);*/

    gyroYaw = NavX.getGyroYaw();
    turretAngle = turret.getRobotRelAngle();

    //TODO: add debouncer here
    if (LimelightGoal.getTargetFound()) { //runs when the LL can see the target
      actualDistance = LimelightGoal.getTargetDistance();
      x = LimelightGoal.getX();
      
      RobotState.updateOdometryWithVision(actualDistance, gyroYaw, turretAngle, x);
    }

    robotPose = RobotState.getPoseEstimate();
    //robotToGoal = new Transform2d(robotPose, goalPose);
    //distance = Math.sqrt(Math.pow(robotToGoal.getX(), 2) + Math.pow(robotToGoal.getY(), 2)); 
    actualDistance = Constants.goalLocation.getDistance(robotPose.getTranslation());
    //TODO: get actual tx based on pose estimate?
    //difference between turret pose and 
    //it should be the same though if the rest of this works right?

    angleToTarget = Units.radiansToDegrees(Math.atan2(
      robotPose.getY() - Constants.goalLocation.getY(), 
      robotPose.getX() - Constants.goalLocation.getX())) - 180;

    //velocities with respect to target
    //paraV = mecDriveTrain.getParaV(angleToTarget - gyroYaw);
    //perpV = mecDriveTrain.getPerpV(angleToTarget - gyroYaw);
    vx = RobotState.getGlobalMecVx();
    vy = RobotState.getGlobalMecVy();
    angV = NavX.getGyroAngV();


    effectiveDistance = RobotState.getEffectiveDistance();
    offsetAngle = RobotState.getOffsetAngleDeg(effectiveDistance);

    turret.setMotorPosPID(robotPose, angleToTarget + offsetAngle - robotPose.getRotation().getDegrees(), vx, vy, angV, effectiveDistance);
    //hood.setMotorPosPID(effectiveDistance, 0);
    //shooter.setMotorsVelPID(effectiveDistance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
    shooter.stopMotors();
    hood.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
