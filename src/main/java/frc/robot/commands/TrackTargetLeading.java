// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.library.Data;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightGoal;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/* tracks target like TrackTargetStationary command but accounts for the robots movement by leading the shot
so that you can shoot while moving*/
public class TrackTargetLeading extends CommandBase {
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
  double distance = 3, x = 0, y = 0, lastX = 0, lastY = 0;

  double gyroYaw = 0;

  double turretAngle = 0;
  //boolean isAllianceBall = true;

  //leading stuff
  double offsetAngle = 0;
  double predictedDistance = 3;
  double airTime = 3;
  double temp = 0;

  /** Creates a new TrackTargetLeading. */
  public TrackTargetLeading(MecDriveTrain mecDriveTrain, Turret turret, Hood hood, Shooter shooter) {
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightGoal.getTargetFound()) { //runs when the LL can see the target
      distance = LimelightGoal.getTargetDistance();
      x = LimelightGoal.getX();

      gyroYaw = NavX.getGyroYaw();
      turretAngle = turret.getAngle();

      //velocities with respect to target
      paraV = mecDriveTrain.getParaV(turretAngle);
      perpV = mecDriveTrain.getPerpV(turretAngle);

      vx = mecDriveTrain.getVx();
      vy = mecDriveTrain.getVy();
      
      angV = mecDriveTrain.getAngV();

      temp = airTime * (vy * Math.sin(gyroYaw + turretAngle - x) + vx * Math.cos(gyroYaw + turretAngle - x));

      offsetAngle = Math.asin(temp / predictedDistance);
      predictedDistance = temp / Math.sin(offsetAngle);
      //predictedDistance = distance / Math.cos(Math.toRadians(offsetAngle)); //something like this

      //desiredHoodAngle = -3 * distance + 85; //degrees
      //desiredTurretV = x / 100; //- 3 * perpV;

      hood.setMotorPosPID(predictedDistance, paraV);

      //turret.setMotorPosPID(x - offsetAngle, perpV, predictedDistance, angV); 

      //shooter.shootWithInitialBallVelocity(paraV, perpV, desiredHoodAngle, desiredTurretAngle, distance);
      //shooter.setMotorsVelPID(predictedDistance);

      airTime = Data.getAirtime(predictedDistance);
    }
    else //this runs when the target is not in view of camera
    {}
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
