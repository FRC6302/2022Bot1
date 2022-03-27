// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightGoal;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/*command that tracks target but does not account for leading the shot, so in theory it would work if you 
completely brake and then instantly shoot. It shouldn't need time to adjust after stopping*/
public class TrackTargetCenter extends CommandBase {
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

  //double gyroYaw = 0;

  double turretAngle = 0;
  //boolean isAllianceBall = true;

  double offsetAngle = 0;
  //double effectiveDistance = 3;

  
  /** Creates a new TrackTarget. */
  public TrackTargetCenter(MecDriveTrain mecDriveTrain, Turret turret,/* Hood hood,*/ Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
    this.turret = turret;
    //this.hood = hood;
    this.shooter = shooter;

    //not adding drivetrain to requirements bc that would not let us drive?
    addRequirements(turret,/* hood,*/ shooter);
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

      //gyroYaw = NavX.getGyroYaw();
      turretAngle = turret.getAngle();

      //velocities with respect to target
      paraV = mecDriveTrain.getParaV(turretAngle - x);
      perpV = mecDriveTrain.getPerpV(turretAngle - x);
      
      angV = mecDriveTrain.getAngV();

      //offsetAngle = 10 * perpV / distance;
      //effectiveDistance = distance / Math.cos(Math.toRadians(offsetAngle)); //something like this

      //isAllianceBall = ColorSensor.getLastestBallIsAlliance();

      //desiredHoodAngle = -3 * distance + 85; //degrees
      //desiredTurretV = x / 100; //- 3 * perpV;

      //hood.setMotorPosPID(distance, paraV);

      //turret.setMotorPosPID(x, perpV, distance, 0); 

      //shooter.shootWithInitialBallVelocity(paraV, perpV, desiredHoodAngle, desiredTurretAngle, distance);
      //shooter.setMotorsVelPID(distance);
    }
    else //this runs when the target is not in view of camera
    {
      paraV = mecDriveTrain.getParaV(turretAngle);
      distance = LimelightGoal.getTargetDistance();
      /*the distance value here is calculated internally from lastX and lastY, so it doesnt matter that
      the target isnt in view*/
      //shooter.setMotorsVelPID(distance);
      //hood.setMotorPosPID(distance, paraV);
      
      /*if target is out of view, x and y will default to zero, so we use the last values of them
      from before they went out of sight to guess how to move to get the target back in view*/
      lastX = LimelightGoal.getLastX();
      lastY = LimelightGoal.getLastY();
      if (lastX < -29) { //happens when we turn too far to the right
        turret.setMotor(Constants.turretSeekSpeed);
      }
      else if (lastX > 29) { //happens when we turn too far to the left
        turret.setMotor(-Constants.turretSeekSpeed);
      }
      else if (Math.abs(lastY) > 24) { //happens when we got too close or far and target is out of view.
        turret.setMotor(0); //because target is out of view vertically, rotating turret wont change anything
      }
      else{ 
        /*happens when the target was not on the edge of field of view when target become out of sight,
        which would only happen if the LL disconnected or something */
        turret.setMotor(0);

        for (int i = 0; i < 2; i++) { //repeated to emphasize to driver
          DriverStation.reportWarning("LIMELIGHT CANNOT FIND TARGET, CHECK CONNECTION", false);
        }
      }
      
    }

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