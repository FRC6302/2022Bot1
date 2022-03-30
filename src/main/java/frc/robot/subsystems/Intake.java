// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax motorIntake;
  private RelativeEncoder encIntake;


  /** Creates a new Intake. */
  public Intake() {
    motorIntake = new CANSparkMax(Constants.motorIntake, MotorType.kBrushless);

    motorIntake.restoreFactoryDefaults();
    motorIntake.setIdleMode(IdleMode.kBrake);
    motorIntake.setInverted(false);
    motorIntake.setCANTimeout(Constants.sparkCANTimeoutMs);

    encIntake = motorIntake.getEncoder();

    motorIntake.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake pos", encIntake.getPosition());
  }

  public void setMotor(double speed) {
    motorIntake.set(speed);
  }
  
  public void setMotor() {
    motorIntake.setVoltage(Constants.intakeDefaultVolts);
  }

  public void stopMotor() {
    motorIntake.set(0);
  }


}
