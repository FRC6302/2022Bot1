// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Feeders;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightGoal;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/*command that tracks target but does not account for leading the shot, so in theory it would work if you 
completely brake and then instantly shoot. It won't need time to adjust after stopping*/
public class TrackTargetCenterPose extends CommandBase {
  MecDriveTrain mecDriveTrain;
  Turret turret;
  Hood hood;
  Shooter shooter;
  Feeders feeders;

  //outputs of command
  double desiredTurretV = 0, desiredTurretAngle = 0;
  double hoodAdjust = 0, desiredHoodAngle = 0;

  //from chassis
  double paraV = 0, perpV = 0, angV = 0, vx = 0, vy = 0;
  
  //from limelight
  double distance = 3, x = 0, y = 0, lastX = 0, lastY = 0;

  double gyroYaw = 0;

  double turretAngle = 0;
  double angleToTarget = 0;

  boolean isAllianceBall = true;

  //needed for leading the shot
  //double offsetAngle = 0; 
  //double effectiveDistance = 3;

  //private final Pose2d goalPose = new Pose2d(0, 0, new Rotation2d());
  //private Pose2d robotPose = new Pose2d(Constants.goalLocation, new Rotation2d());
  private Pose2d robotPose;
  //private Transform2d robotToGoal = new Transform2d();

  
  //private Translation2d robotLocation = new Translation2d();
  double estimatedDistance = 3;

  boolean updateVisionOdom = true;
  boolean isAuton = false;

  boolean missTarget = false;

  
  /** Creates a new TrackTargetCenterPose. */
  public TrackTargetCenterPose(boolean updateVisionOdom, boolean isAuton, MecDriveTrain mecDriveTrain, 
    Turret turret, Hood hood, Shooter shooter, Feeders feeders) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    this.feeders = feeders;
    this.updateVisionOdom = updateVisionOdom;
    this.isAuton = isAuton;

    if (isAuton) {
      addRequirements(turret, hood, shooter);
    }
    else {
      addRequirements(turret, hood, shooter, feeders);
    }
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose = RobotState.getPoseEstimate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroYaw = NavX.getGyroYaw();
    turretAngle = turret.getRobotRelAngle();

    if (LimelightGoal.getTargetFound()) { //runs when the LL can see the target
      distance = LimelightGoal.getTargetDistance();
      x = LimelightGoal.getX();
      
      if (updateVisionOdom) {
        RobotState.updateOdometryWithVision(distance, gyroYaw, turretAngle, x);
      }
      
    }

    robotPose = RobotState.getPoseEstimate();
    //robotToGoal = new Transform2d(robotPose, goalPose);
    //distance = Math.sqrt(Math.pow(robotToGoal.getX(), 2) + Math.pow(robotToGoal.getY(), 2)); 
    distance = Constants.goalLocation.getDistance(robotPose.getTranslation());
    //get actual tx based on pose estimate?
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

    //effectiveDistance = distance / Math.cos(Math.toRadians(offsetAngle)); //something like this

    //isAllianceBall = ColorSensor.getCurrentBallIsAlliance();
    //if we pick up the wrong color ball, we wanna shoot it out in a way that misses the goal on purpose but stays on the field
    /*if (Timer.getFPGATimestamp() > Constants.timeToShootOppositeBall + 1 && ColorSensor.getTimeSinceOppositeBall() < Constants.timeToShootOppositeBall) {
      shooter.missTarget();
      hood.missTarget();
      turret.setMotorPosPID(-Constants.turretOffsetForMissing, 0, 7, 0); 

      //makes sure the turret doesnt try to offset the ball into arm. Adding plus 10 for leeway
      /*if (angleToTarget + Constants.turretOffsetForMissing + 10 < Constants.maxTurretAngle) {
        turret.setMotorPosPID(angleToTarget + Constants.turretOffsetForMissing - gyroYaw, distance, perpV, angV); 
      }
      else { //if it cant offset to the left, then do it to the right instead
        turret.setMotorPosPID(angleToTarget - Constants.turretOffsetForMissing - gyroYaw, distance, perpV, angV); 
      }*/
    
    //else {
      //hood.setMotorPosPID(7, 0);
      //turret.setMotorPosPID(0, 7, 0, 0); 
      //shooter.setMotorsDefaultVolts();
    //}

    /*if (Robot.robotContainer.getOperatorButton(Constants.missTargetButton).getAsBoolean()) {
      
    }*/


    
    hood.setMotorPosPID(distance, 0);
    turret.setMotorPosPID(robotPose, angleToTarget - robotPose.getRotation().getDegrees(), vx, vy, angV, distance);
    shooter.setMotorsVelPID(distance);

    if (!isAuton) {
      if (turret.getIsResetting()) {
        feeders.stopBothMotors();
      }
      else {
        feeders.setBothMotors(0.5, 0.8);
      }
    }
    

    //shooter.shootWithInitialBallVelocity(paraV, perpV, desiredHoodAngle, desiredTurretAngle, distance);
    //shooter.setMotorsVelPID(distance);
    

    //SmartDashboard.putNumber("vision pose x", robotPose.getX());
    //SmartDashboard.putNumber("vision pose y", robotPose.getY());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
    shooter.stopMotors();
    hood.stopMotor();

    feeders.stopBothMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void setMissTarget(boolean miss) {
    missTarget = true;
  }
}
