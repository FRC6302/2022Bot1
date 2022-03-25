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

public class Climbers extends SubsystemBase {
  private CANSparkMax motorFirstClimber;
  private CANSparkMax motorSecondClimber;

  private RelativeEncoder encFirstClimber;
  private RelativeEncoder encSecondClimber;

  private ProfiledPIDController pidController = new ProfiledPIDController(Constants.kpPosClimb, 0, 0, 
    new Constraints(Constants.maxClimbV, (Constants.maxClimbA)));

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    Constants.ksClimb, Constants.kvClimb, Constants.kaClimb);

  private final double posFactor = 1;
  private final double velFactor = 1;

  /** Creates a new Climber. */
  public Climbers() {
    motorFirstClimber = new CANSparkMax(Constants.motorFirstClimber, MotorType.kBrushless);
    motorSecondClimber = new CANSparkMax(Constants.motorSecondClimber, MotorType.kBrushless);

    motorFirstClimber.restoreFactoryDefaults();
    motorSecondClimber.restoreFactoryDefaults();
    //motorFirstClimber.enableVoltageCompensation(12.5);

    motorFirstClimber.setInverted(false);
    motorSecondClimber.setInverted(false);

    motorFirstClimber.setIdleMode(IdleMode.kCoast);
    motorSecondClimber.setIdleMode(IdleMode.kCoast);

    encFirstClimber = motorFirstClimber.getEncoder();
    encSecondClimber = motorSecondClimber.getEncoder();

    encFirstClimber.setPositionConversionFactor(posFactor);
    encSecondClimber.setPositionConversionFactor(posFactor);

    //encRightClimber.setVelocityConversionFactor(velFactor);
    //encLeftClimber.setVelocityConversionFactor(velFactor);

    motorFirstClimber.burnFlash();
    motorSecondClimber.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("first climb enc pos", encFirstClimber.getPosition());
    SmartDashboard.putNumber("second climb enc pos", encSecondClimber.getPosition());
  }

  public void setMotorsPosPID(double targetPosition) {

  }

  public void setMotors(double firstSpeed, double secondSpeed) {
    motorFirstClimber.set(firstSpeed);
    motorSecondClimber.set(secondSpeed);
    
  }

  public void stopMotors() {
    motorFirstClimber.set(0);
    motorSecondClimber.set(0);
  }
}
