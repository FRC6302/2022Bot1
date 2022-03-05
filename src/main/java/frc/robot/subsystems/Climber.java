// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private CANSparkMax motorRightClimber;
  private CANSparkMax motorLeftClimber;

  private RelativeEncoder encRightClimber;
  private RelativeEncoder encLeftClimber;

  private ProfiledPIDController pidController = new ProfiledPIDController(Constants.kpPosClimb, 0, 0, 
    new Constraints(Constants.maxClimbV, (Constants.maxClimbA)));

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    Constants.ksClimb, Constants.kvClimb, Constants.kaClimb);

  private final double posFactor = 1;
  private final double velFactor = 1;

  /** Creates a new Climber. */
  public Climber() {
    motorRightClimber = new CANSparkMax(Constants.motorRightClimber, MotorType.kBrushless);
    motorLeftClimber = new CANSparkMax(Constants.motorLeftClimber, MotorType.kBrushless);

    motorRightClimber.restoreFactoryDefaults();
    motorLeftClimber.restoreFactoryDefaults();
    //motorRightClimber.enableVoltageCompensation(12.5);

    motorRightClimber.setInverted(false);
    motorLeftClimber.setInverted(false);

    motorRightClimber.setIdleMode(IdleMode.kBrake);
    motorLeftClimber.setIdleMode(IdleMode.kBrake);

    encRightClimber = motorRightClimber.getEncoder();
    encLeftClimber = motorLeftClimber.getEncoder();

    encRightClimber.setPositionConversionFactor(posFactor);
    encLeftClimber.setPositionConversionFactor(posFactor);

    encRightClimber.setVelocityConversionFactor(velFactor);
    encLeftClimber.setVelocityConversionFactor(velFactor);

    motorRightClimber.burnFlash();
    motorLeftClimber.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("right climb enc pos", encRightClimber.getPosition());
    SmartDashboard.putNumber("left climb enc pos", encLeftClimber.getPosition());
  }

  public void setMotorsPosPID(double targetPosition) {

  }
}
