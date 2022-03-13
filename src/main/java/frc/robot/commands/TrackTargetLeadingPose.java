// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.library.Data;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightGoal;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.NavX;
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
  double desiredTurretV = 0, desiredTurretAngle = 0;
  double hoodAdjust = 0, desiredHoodAngle = 0;

  //from chassis
  double paraV = 0, perpV = 0, angV = 0, vx = 0, vy = 0;
  
  //from limelight
  double rawDistance = 3, x = 0, y = 0, lastX = 0, lastY = 0;

  double gyroYaw = 0;

  double turretAngle = 0;
  //boolean isAllianceBall = true;

  //leading stuff
  double offsetAngle = 0; //radians
  double effectiveDistance = 3; //meters
  double actualDistance;
  double airTime = 3; //seconds
  double temp = 0;

  //approximating derivative stuff
  double prevOffset = offsetAngle, offsetDerivative = 1;
  double prevAirtime = airTime, airTimeDerivative = 1;
  double effectiveDistanceDerivative = 1, effectiveDistancePrediction = effectiveDistance;

  //pose stuff
  private Pose2d robotPose = new Pose2d();
  //private Transform2d robotToGoal = new Transform2d();
  //private Translation2d robotLocation = new Translation2d();
  //double estimatedActualDistance = 0;


  /** Creates a new TrackTargetLeading. */
  public TrackTargetLeadingPose(MecDriveTrain mecDriveTrain, Turret turret, Hood hood, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;

    //not adding drivetrain to requirements bc that would not let us drive?
    addRequirements(turret, hood, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*airTime = Data.getAirtime(3);

    temp = airTime * (vy * (robotPose.getX() - Constants.goalLocation.getX()) 
      + vx * (robotPose.getY() - Constants.goalLocation.getY()));
    offsetAngle = Math.asin(temp / effectiveDistance);*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroYaw = NavX.getGyroYaw();
    turretAngle = turret.getAngle();

    if (LimelightGoal.getTargetFound()) { //runs when the LL can see the target
      rawDistance = LimelightGoal.getTargetDistance();
      x = LimelightGoal.getX();
      
      mecDriveTrain.updateOdometryWithVision(rawDistance, gyroYaw, turretAngle, x);
    }

    robotPose = mecDriveTrain.getPoseEstimate();
    actualDistance = robotPose.getTranslation().getDistance(Constants.goalLocation);
    //robotToGoal = new Transform2d(robotPose, goalPose);
    //distance = Math.sqrt(Math.pow(robotToGoal.getX(), 2) + Math.pow(robotToGoal.getY(), 2)); 

    //velocities with respect to target
    paraV = mecDriveTrain.getParaV(turretAngle);
    perpV = mecDriveTrain.getPerpV(turretAngle);

    vx = mecDriveTrain.getVx();
    vy = mecDriveTrain.getVy();
    
    angV = mecDriveTrain.getAngV();

    //guess airtime 0.020 sec into future? linear approximation?
    // airtime = airtime + 0.020 * derivative

    temp = airTime * (vy * (robotPose.getX() - Constants.goalLocation.getX()) 
      + vx * (robotPose.getY() - Constants.goalLocation.getY()));

    //temp = airTime * (vy * Math.sin(gyroYaw + turretAngle - x) + vx * Math.cos(gyroYaw + turretAngle - x));
    //TODO: compare these temp values and make sure they are close

    //change so it doesn't use the effective distance from previous loop
    //offsetAngle = Math.asin(temp / effectiveDistance);
    effectiveDistance = temp / (actualDistance * Math.sin(offsetAngle));

    //take derivate of effective distance formula to get distance adjustment approximation to plug into airtime. a = 0?

    //predictedDistance = distance / Math.cos(Math.toRadians(offsetAngle)); //something like this

    //desiredHoodAngle = -3 * distance + 85; //degrees

    hood.setMotorPosPID(effectiveDistance, paraV);

    turret.setMotorPosPID(x - Units.radiansToDegrees(offsetAngle), perpV, effectiveDistance, angV); 

    //shooter.shootWithInitialBallVelocity(paraV, perpV, desiredHoodAngle, desiredTurretAngle, distance);
    //shooter.setMotorsVelPID(predictedDistance);

    prevOffset = offsetAngle;
    prevAirtime = airTime;

    //effectiveDistance += 0.020 * derivative gives the effective distance in 0.020 seconds.
    //then you can get the air time and offset angle for the next loop
    airTime = Data.getAirtime(effectiveDistance);

    //these derivatives are very approximate
    offsetDerivative = (offsetAngle - prevOffset) / 2.;
    airTimeDerivative = (airTime - prevAirtime) / 2.;

    //updates airtime based on approximate of 
    effectiveDistanceDerivative = airTimeDerivative * effectiveDistance / airTime; 

    //predicts what the effective distance will be in 20 ms when the loop run again
    effectiveDistancePrediction = effectiveDistance + 0.020 * effectiveDistanceDerivative;
    //double temp2 = temp / airTime;
    airTime = Data.getAirtime(effectiveDistancePrediction); 
    offsetAngle = Math.asin((airTime * temp / prevAirtime) / effectiveDistancePrediction);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
