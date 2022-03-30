// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.MecDriveTrain;

public class DriveMec extends CommandBase {
  private MecDriveTrain mecDriveTrain;

  double x = 0, y = 0, z = 0;

  /** Creates a new DriveMec. */
  public DriveMec(MecDriveTrain mecDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    z = -1.5 * (Robot.robotContainer.getDriverDeadzoneAxis(Constants.rightTrigger)
    - Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftTrigger));
    x = -1.5 * Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftStickY);
    y = -1.5 * Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftStickX);

    SmartDashboard.putNumber("forward input joystick", x);
    SmartDashboard.putNumber("sideways input joystick", y);
    //SmartDashboard.putNumber("perpV", mecDriveTrain.getPerpV(Limelight.getX()));
    //SmartDashboard.putNumber("paraV", mecDriveTrain.getParaV(Limelight.getLastX()));

    //mecDriveTrain.setMotorsSimple(x, y, z);

    //mecDriveTrain.setMotors(x, y, z, true); 
    mecDriveTrain.setMotorsFieldRel(x, y, z);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mecDriveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
