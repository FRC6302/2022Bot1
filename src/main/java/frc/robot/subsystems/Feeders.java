// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeders extends SubsystemBase {
  private WPI_TalonSRX motorFeederFront = new WPI_TalonSRX(Constants.motorFeederFront);
  private WPI_TalonSRX motorFeederMiddle = new WPI_TalonSRX(Constants.motorFeederMiddle);


  /** Creates a new Feeders. */
  public Feeders() {
    motorFeederFront.setInverted(false);
    motorFeederMiddle.setInverted(false);

    motorFeederFront.setNeutralMode(NeutralMode.Brake);
    motorFeederMiddle.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
