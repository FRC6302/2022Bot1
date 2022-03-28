// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.sound.sampled.Line;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.library.Data;

public class Shooter extends SubsystemBase {
  //private WPI_TalonSRX motorShooterTop = new WPI_TalonSRX(Constants.motorShooterTop);
  //private WPI_TalonSRX motorShooterBottom = new WPI_TalonSRX(Constants.motorShooterBottom);

  private CANSparkMax motorShooterTop;
  private CANSparkMax motorShooterBottom;

  /*private Encoder topShooterEncoder = new Encoder(Constants.topShooterEncA, Constants.topShooterEncB, 
    false, EncodingType.k4X);
  private Encoder bottomShooterEncoder = new Encoder(Constants.bottomShooterEncA, Constants.bottomShooterEncB, 
    true, EncodingType.k4X);*/

  private RelativeEncoder topShooterEncoder;
  private RelativeEncoder bottomShooterEncoder;

  //private BangBangController bangBangTop = new BangBangController(0.05);
  //private BangBangController bangBangBottom = new BangBangController(0.05);

  // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(
    Constants.ksTopShooter, Constants.kVTopShooter, Constants.kATopShooter);
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(
    Constants.ksBottomShooter, Constants.kVBottomShooter, Constants.kABottomShooter);
  
  //private ProfiledPIDController topPID = new ProfiledPIDController(Constants.kpTopShooter, 0, 0, 
    //new Constraints(Constants.maxShooterV, Constants.maxShooterA));
  //private ProfiledPIDController bottomPID = new ProfiledPIDController(Constants.kpBottomShooter, 0, 0, 
    //new Constraints(Constants.maxShooterV, Constants.maxShooterA));
  private PIDController topPID = new PIDController(Constants.kpTopShooter, 0, 0);
  private PIDController bottomPID = new PIDController(Constants.kpBottomShooter, 0, 0);

  //distance per pulse = pi * (wheel diameter / counts per revolution) / gear reduction between encoder and shaft
  //rev through bore encoder is 8192 counts per rev?
  //ctre mag encoder is 4096
  private final double distancePerPulse = Math.PI * 0.1524; // / 4096) / 1; //blue wheel diam is 6 inches or 0.1524 meters


  // The plant holds a state-space model of our flywheel. This system has the following properties:
  // States: [velocity], in meters per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in meters per second.
  private final LinearSystem<N1, N1, N1> topPlant = LinearSystemId.identifyVelocitySystem(
    Constants.kVTopShooter, Constants.kATopShooter);
  private final LinearSystem<N1, N1, N1> bottomPlant = LinearSystemId.identifyVelocitySystem(
      Constants.kVBottomShooter, Constants.kABottomShooter);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> topObserver = new KalmanFilter<>(
    Nat.N1(),
    Nat.N1(),
    topPlant,
    VecBuilder.fill(3), // How accurate we think our model is, standard deviation in m/s
    VecBuilder.fill(.01), // How accurate we think our encoder data is, standard deviation in m/s
    Constants.loopTime
  );
  private final KalmanFilter<N1, N1, N1> bottomObserver = new KalmanFilter<>(
    Nat.N1(),
    Nat.N1(),
    topPlant,
    VecBuilder.fill(3), // How accurate we think our model is, standard deviation in m/s
    VecBuilder.fill(.01), // How accurate we think our encoder data is, standard deviation in m/s
    Constants.loopTime
  );

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> topController = new LinearQuadraticRegulator<>(
    topPlant,
    VecBuilder.fill(8), // qelms. Velocity error tolerance, in meters per second. Decrease
    // this to more heavily penalize state excursion, or make the controller behave more
    // aggressively.
    VecBuilder.fill(6.0), // relms. Control effort (voltage) tolerance. Decrease this to more
    // heavily penalize control effort, or make the controller less aggressive. 12 is a good
    // starting point because that is the (approximate) maximum voltage of a battery.
    Constants.loopTime
  ); 
  private final LinearQuadraticRegulator<N1, N1, N1> bottomController = new LinearQuadraticRegulator<>(
    bottomPlant,
    VecBuilder.fill(8), // qelms. Velocity error tolerance, in meters per second. Decrease
    // this to more heavily penalize state excursion, or make the controller behave more
    // aggressively.
    VecBuilder.fill(6.0), // relms. Control effort (voltage) tolerance. Decrease this to more
    // heavily penalize control effort, or make the controller less aggressive. 12 is a good
    // starting point because that is the (approximate) maximum voltage of a battery.
    Constants.loopTime
  ); 

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> topLoop = new LinearSystemLoop<>(
    topPlant, topController, topObserver, 12.0, Constants.loopTime);
  private final LinearSystemLoop<N1, N1, N1> bottomLoop = new LinearSystemLoop<>(
    bottomPlant, bottomController, bottomObserver, 12.0, Constants.loopTime);

  //estimating shooter velocity stuff
  LinearFilter topVelocityFilter = LinearFilter.movingAverage(Constants.velocityPeriodsToAverage);
  LinearFilter bottomVelocityFilter = LinearFilter.movingAverage(Constants.velocityPeriodsToAverage);
  //MedianFilter topVelocityFilter = new MedianFilter(Constants.velocityPeriodsToAverage);
  //MedianFilter bottomVelocityFilter = new MedianFilter(Constants.velocityPeriodsToAverage);
  private double topPrevPos = 0, bottomPrevPos = 0, prevTime = 0;
  private double averagedTopV = 0, averagedBottomV = 0;

  /** Creates a new Shooter. */
  public Shooter() {

    /*motorShooterTop.configFactoryDefault();
    motorShooterBottom.configFactoryDefault();

    //DO NOT CHANGE OR BANG BANG CONTROL WILL NOT WORK AND SHOOTER WILL BREAK
    motorShooterTop.setNeutralMode(NeutralMode.Coast);
    motorShooterBottom.setNeutralMode(NeutralMode.Coast);

    //motorShooterTop.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    //motorShooterBottom.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    motorShooterTop.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    motorShooterBottom.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    motorShooterTop.setSensorPhase(false);
    motorShooterBottom.setSensorPhase(false);
    
    motorShooterTop.setInverted(false);
    motorShooterBottom.setInverted(true);*/
    
    //topShooterEncoder.setDistancePerPulse(distancePerPulse);
    //bottomShooterEncoder.setDistancePerPulse(distancePerPulse);

    motorShooterTop = new CANSparkMax(Constants.motorShooterTop, MotorType.kBrushless);
    motorShooterBottom = new CANSparkMax(Constants.motorShooterBottom, MotorType.kBrushless);

    motorShooterTop.restoreFactoryDefaults();
    motorShooterBottom.restoreFactoryDefaults();

    motorShooterTop.setInverted(true);
    motorShooterBottom.setInverted(false);

    //DO NOT CHANGE TO BRAKE OR BANG BANG CONTROL WILL NOT WORK AND SHOOTER WILL BREAK
    motorShooterTop.setIdleMode(IdleMode.kCoast);
    motorShooterBottom.setIdleMode(IdleMode.kCoast);

    topShooterEncoder = motorShooterTop.getEncoder();
    bottomShooterEncoder = motorShooterBottom.getEncoder();

    //saves the settings to the motors
    motorShooterTop.burnFlash();
    motorShooterBottom.burnFlash();

    //these might mess things up
    topLoop.reset(VecBuilder.fill(0));
    bottomLoop.reset(VecBuilder.fill(0));

    topVelocityFilter.reset();
    bottomVelocityFilter.reset();
  }

  @Override
  public void periodic() {
    updateVelocities();


    // This method will be called once per scheduler run
    SmartDashboard.putNumber("top shooter pos", getTopEncPos());
    SmartDashboard.putNumber("bottom shooter pos", getBottomEncPos());

    SmartDashboard.putNumber("top shooter vel", averagedTopV);
    SmartDashboard.putNumber("bottom shooter vel", averagedBottomV);

    SmartDashboard.putNumber("top shooter intern vel", getTopShooterEncVel());
    SmartDashboard.putNumber("bottom shooter intern vel", getBottomShooterEncVel());

    //output averaged velocity and then encoder delayed v and see the delay in the values. If there is one then you did it right
    
  }

  

  public void setMotorsStateSpace(double distance) {
    double topSetpoint = 0;// Data.getTopShooterVoltage(distance);
    double bottomSetpoint = 0;// Data.getBottomShooterVoltage(distance);

    // Sets the target speed of our flywheel. This is similar to setting the setpoint of a PID controller.
    topLoop.setNextR(topSetpoint);
    bottomLoop.setNextR(bottomSetpoint);

    // Correct our Kalman filter's state vector estimate with encoder data.
    topLoop.correct(VecBuilder.fill(averagedTopV));
    bottomLoop.correct(VecBuilder.fill(averagedBottomV));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next state with out Kalman filter.
    topLoop.predict(Constants.loopTime);
    bottomLoop.predict(Constants.loopTime);

    // Send the new calculated voltage to the motors.
    //motorShooterTop.setVoltage(topLoop.getU(0));
    motorShooterBottom.setVoltage(bottomLoop.getU(0));

    //graph these + enc data on glass in same graph and tune the controller std dev values
    SmartDashboard.putNumber("top shooter state estimate", topLoop.getU(0));
    SmartDashboard.putNumber("bottom shooter state estimate", bottomLoop.getU(0));
    SmartDashboard.putNumber("top shooter setpoint", topSetpoint);
    SmartDashboard.putNumber("bottom shooter setpoint", bottomSetpoint);
    
  }


  public void setMotorsVelPID(double distance) {
    //double setpointV = Data.getTopShooterVoltage(distance);
    double topV = Data.getTopShooterVel(distance);
    double bottomV = 1.25 * Data.getBottomShooterVel(distance); //bottom going 80% of commanded speed

    motorShooterTop.setVoltage(topPID.calculate(getTopShooterEncVel(), topV) + topFeedforward.calculate(topV));
    motorShooterBottom.setVoltage(bottomPID.calculate(getTopShooterEncVel(), bottomV) + bottomFeedforward.calculate(bottomV));
  }

  /*public void setMotorsVelPID(double distance, double perpV, double paraV) {
    double setpointV = Data.getTopShooterVoltage(distance);
    
    //break setpointV into component parts
    //add perpV and paraV to their respective components
    //combine components into new setpointV
  }*/

  public void setMotors(double speed) {
    //motorShooterTop.set(ControlMode.PercentOutput, speed);
    //motorShooterBottom.set(ControlMode.PercentOutput, speed);

    motorShooterTop.set(speed);
    motorShooterBottom.set(speed);
  }
  
  public void setMotors(double topSpeed, double bottomSpeed) {
    //motorShooterTop.set(ControlMode.PercentOutput, topSpeed);
    //motorShooterBottom.set(ControlMode.PercentOutput, bottomSpeed);

    motorShooterTop.set(topSpeed);
    motorShooterBottom.set(bottomSpeed);
  }

  public void setMotorVolts(double topVolts, double bottomVolts) {
    motorShooterTop.setVoltage(topVolts);
    motorShooterBottom.setVoltage(bottomVolts);
  }

  public void setTopMotor(double speed) {
    motorShooterTop.set(speed);
  }

  public void setBottomMotor(double speed) {
    motorShooterBottom.set(speed);
  }

  public void updateVelocities() {
    //averages the last few velocities calculated from the encoder position data
    averagedTopV = topVelocityFilter.calculate((getTopEncPos() - topPrevPos) / (Timer.getFPGATimestamp() - prevTime));
    averagedBottomV = bottomVelocityFilter.calculate((getBottomEncPos() - bottomPrevPos) / (Timer.getFPGATimestamp() - prevTime));


    topPrevPos = getTopEncPos();
    bottomPrevPos = getBottomEncPos();
    prevTime = Timer.getFPGATimestamp();

    //NetworkTableInstance.getDefault().flush();
  }

  public double getTopShooterEncVel() {
    //multiplying by 10 because to turn 100 ms to 1 sec because it reports with per 100 ms units
    //return 10.0 * motorShooterTop.getSelectedSensorVelocity() * distancePerPulse;

    return topShooterEncoder.getVelocity() * 2 * Math.PI / 60;
    //return 0;
    //return averagedTopV;
  }

  public double getBottomShooterEncVel() {
    //return 10.0 * motorShooterBottom.getSelectedSensorVelocity() * distancePerPulse;

    return bottomShooterEncoder.getVelocity() * 2 * Math.PI / 60;
    //return 0;
    //return averagedBottomV;
  }
  
  private double getTopEncPos() {
    return topShooterEncoder.getPosition() * 2 * Math.PI;
  }

  private double getBottomEncPos() {
    return bottomShooterEncoder.getPosition() * 2 * Math.PI;
  }


  /*public void setWithBangBang(double desiredTop, double desiredBottom) {
    // Controls a motor with the output of the BangBang controller
    setMotors(
      bangBangTop.calculate(getTopShooterEncRate(), desiredTop), 
      bangBangBottom.calculate(getBottomShooterEncRate(), desiredBottom));
  }


  public void setWithBangBangAndFeedForward(double desiredTop, double desiredBottom) {
    setMotors(
      bangBangTop.calculate(getTopShooterEncRate(), desiredTop) 
        + 0.9 * topFeedforward.calculate(desiredTop),
      bangBangBottom.calculate(getBottomShooterEncRate(), desiredBottom) 
        + 0.9 * bottomFeedforward.calculate(desiredBottom));
  }*/

  public void stopMotors() {
    setMotors(0);
  }

  public void setMotorsDefaultVolts() {
    setMotorVolts(Constants.topShooterDefaultVolts, Constants.bottomShooterDefaultVolts);
  }

  //want to give the ball a lot of backspin so that it doesn't go out of the field after it bounces
  public void missTarget() {
    setMotorVolts(Constants.topShooterVoltsForMissing, Constants.bottomShooterVoltsForMissing);
  }

}
