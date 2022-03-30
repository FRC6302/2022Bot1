// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NeoTest extends SubsystemBase {
  private CANSparkMax motorNeoTest = new CANSparkMax(Constants.motorNeoTest, MotorType.kBrushless);

  private RelativeEncoder encNeoTest;

  //private Encoder encNeoTest = new Encoder(Constants.encTestA, Constants.encTestB, false, EncodingType.k4X);

  /** Creates a new NeoTest. */
  public NeoTest() {
    motorNeoTest.restoreFactoryDefaults();
    motorNeoTest.setInverted(false);
    motorNeoTest.setIdleMode(IdleMode.kCoast);
    //motorNeoTest.setSoftLimit(SoftLimitDirection.kForward, 720);*/
    motorNeoTest.setCANTimeout(Constants.sparkCANTimeoutMs);

    encNeoTest = motorNeoTest.getEncoder();
    //encNeoTest.setInverted(false);
    //encNeoTest.setPositionConversionFactor(Math.PI * 0.1524); //6 in = 0.1524 m wheel
    //encNeoTest.setVelocityConversionFactor(Math.PI * 0.1524);

    //encNeoTest.setDistancePerPulse(1);

    motorNeoTest.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("test enc p", encNeoTest.getPosition());
  }

  public void setMotor(double speed) {
    motorNeoTest.set(speed); //values from -1 to 1
  }
}
