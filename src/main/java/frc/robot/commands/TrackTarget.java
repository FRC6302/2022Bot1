// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class TrackTarget extends CommandBase {
  MecDriveTrain mecDriveTrain;
  Turret turret;
  Hood hood;
  Shooter shooter;

  //outputs of command
  double turretAdjust = 0, desiredTurretAngle = 0;
  double hoodAdjust = 0, desiredHoodAngle = 0;

  //from chassis
  double paraV = 0, perpV = 0;
  
  //from limelight
  double distance = 0, x = 0, y = 0, lastX = 0, lastY = 0;

  
  /** Creates a new TrackTarget. */
  public TrackTarget(MecDriveTrain mecDriveTrain, Turret turret, Hood hood, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;

    addRequirements(mecDriveTrain, turret, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Limelight.getTargetFound()) { //if target is found
      distance = Limelight.getTargetDistance();
      x = Limelight.getX();

      paraV = mecDriveTrain.getParaV();
      perpV = mecDriveTrain.getPerpV();

      desiredHoodAngle = -3 * distance + 85;
      hood.setHoodAngle(desiredHoodAngle);

      turretAdjust = x / 100 - 3 * perpV;
      turret.setMotor(turretAdjust);      
      
      shooter.shootWithInitialBallVelocity(paraV, perpV, desiredHoodAngle, desiredTurretAngle, distance);
    }
    else //this runs when the target is not in view of camera
    {
      //maybe base shooter speed on last distance value?
      shooter.setMotors(Constants.defaultShooterSpeed);
      
      lastX = Limelight.getLastX();
      lastY = Limelight.getLastY();
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

        for (int i = 0; i < 3; i++) { //repeated to emphasize to driver
          DriverStation.reportWarning("LIMELIGHT CANNOT FIND TARGET, REASON UNKNOWN", false);
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
