// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PneumaticsTest extends SubsystemBase {
  
  DoubleSolenoid exampleDouble;
  //Compressor compressor;

  /** Creates a new PneumaticsTest. */
  public PneumaticsTest() {
    exampleDouble = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    //compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    //must be set to a position before you can toggle() it back and forth
    exampleDouble.set(kReverse);
    //compressor.enableAnalog(minPressure, maxPressure);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public synchronized void turnOff() {
    exampleDouble.set(kOff);
  }

  public synchronized void setForward(){
    exampleDouble.set(Value.kForward);
  }

  public synchronized void setReverse(){
    exampleDouble.set(Value.kReverse);
  }

  public synchronized void toggle(){
    exampleDouble.toggle();
  }
}
