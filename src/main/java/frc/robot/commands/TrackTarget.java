// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

  double turretAdjust = 0;
  double hoodAdjust = 0;

  double paraV = 0, perpV = 0, distance = 0;

  double desiredHoodAngle = 0, desiredTurretAngle = 0;
  
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
    if (!Limelight.getTargetFound()) { //if target not found
      turret.setMotor(Constants.turretSeekSpeed);
      shooter.setBothMotors(Constants.defaultShooterSpeed);
    }
    else //this runs when the target is in view of camera
    {
      distance = Limelight.getTargetDistance();

      desiredHoodAngle = -3 * distance + 85;
      hood.setHoodAngle(desiredHoodAngle);

      perpV = mecDriveTrain.getPerpV();
      if (Math.abs(perpV) < 0.3) {
        desiredTurretAngle = Limelight.getX() / 100;
        turret.setAngle(desiredTurretAngle);
      }
      else {
        turret.setAngle(perpV * -3);
      }

      paraV = mecDriveTrain.getParaV();
      shooter.shootWithInitialBallVelocity(paraV, perpV, desiredHoodAngle, desiredTurretAngle, distance);
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
