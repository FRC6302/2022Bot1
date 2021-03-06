// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoTest;

//command I made for testing Neos to make sure they weren't broken
public class TestNeo extends CommandBase {
  NeoTest neoTest;

  /** Creates a new TestNeo. */
  public TestNeo(NeoTest neoTest) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.neoTest = neoTest;

    addRequirements(neoTest);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    neoTest.setMotor(0.2);
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
