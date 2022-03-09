// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightGoal;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/*command that tracks target but does not account for leading the shot, so in theory it would work if you 
completely brake and then instantly shoot. It won't need time to adjust after stopping*/
public class TrackTargetCenterPose extends CommandBase {
  MecDriveTrain mecDriveTrain;
  Turret turret;
  //Hood hood;
  Shooter shooter;

  //outputs of command
  double desiredTurretV = 0, desiredTurretAngle = 0;
  double hoodAdjust = 0, desiredHoodAngle = 0;

  //from chassis
  double paraV = 0, perpV = 0, angV = 0;
  
  //from limelight
  double distance = 3, x = 0, y = 0, lastX = 0, lastY = 0;

  double gyroYaw = 0;

  double turretAngle = 0;
  double angleToTarget = 0;

  //boolean isAllianceBall = true;

  //needed for leading the shot
  //double offsetAngle = 0;
  //double effectiveDistance = 3;

  //private final Pose2d goalPose = new Pose2d(0, 0, new Rotation2d());
  private Pose2d robotPose = new Pose2d();
  //private Transform2d robotToGoal = new Transform2d();

  
  //private Translation2d robotLocation = new Translation2d();
  double estimatedDistance = 3;

  
  /** Creates a new TrackTargetStationary. */
  public TrackTargetCenterPose(MecDriveTrain mecDriveTrain, Turret turret,/* Hood hood,*/ Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
    this.turret = turret;
    //this.hood = hood;
    this.shooter = shooter;

    addRequirements(turret, /*hood,*/ shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroYaw = NavX.getGyroYaw();
    turretAngle = turret.getAngle();

    if (LimelightGoal.getTargetFound()) { //runs when the LL can see the target
      distance = LimelightGoal.getTargetDistance();
      x = LimelightGoal.getX();
      
      mecDriveTrain.updateOdometryWithVision(distance, gyroYaw, turretAngle, x);
    }

    robotPose = mecDriveTrain.getPoseEstimate();
    //robotToGoal = new Transform2d(robotPose, goalPose);
    //distance = Math.sqrt(Math.pow(robotToGoal.getX(), 2) + Math.pow(robotToGoal.getY(), 2)); 
    estimatedDistance = Constants.goalLocation.getDistance(robotPose.getTranslation());
    //TODO: get actual tx based on pose estimate?
    //difference between turret pose and 
    //it should be the same though if the rest of this works right?

    angleToTarget = Units.radiansToDegrees(Math.atan2(robotPose.getY() - Constants.goalLocation.getY(),
      robotPose.getX() - Constants.goalLocation.getX()));

    //velocities with respect to target
    paraV = mecDriveTrain.getParaV(angleToTarget - gyroYaw);
    perpV = mecDriveTrain.getPerpV(angleToTarget - gyroYaw);
    
    angV = mecDriveTrain.getAngV();

    //offsetAngle = 10 * perpV / distance;
    //effectiveDistance = distance / Math.cos(Math.toRadians(offsetAngle)); //something like this

    //isAllianceBall = ColorSensor.getLastestBallIsAlliance();

    //desiredHoodAngle = -3 * distance + 85; //degrees
    //desiredTurretV = x / 100; //- 3 * perpV;

    //hood.setMotorPosPID(estimatedDistance, paraV);

    turret.setMotorPosPID(angleToTarget, distance, 0, perpV, angV, gyroYaw); 

    //shooter.shootWithInitialBallVelocity(paraV, perpV, desiredHoodAngle, desiredTurretAngle, distance);
    //shooter.setMotorsVelPID(distance);

    //SmartDashboard.putNumber("vision pose x", robotPose.getX());
    //SmartDashboard.putNumber("vision pose y", robotPose.getY());


    //hood.setHoodAngle(hoodAdjust);
    //turret.setAngle(turretAdjust);
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
