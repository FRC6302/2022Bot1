// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utilities.InterpolatingDouble;
import frc.robot.Utilities.InterpolatingTreeMap;
import frc.robot.Utilities.LinearInterpolator;
import frc.robot.commands.DriveGTA;
import frc.robot.commands.DriveMec;
import frc.robot.commands.DriveMecTrackTarget;
import frc.robot.commands.MissTarget;
import frc.robot.commands.Move;
import frc.robot.commands.PPMecanumControllerCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.PneumaticsTest;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TouchSensor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  private XboxController driverController;
  //public static XboxController operatorController;

  private DriveTrain driveTrain;
  //private DriveGTA driveGTA;

  private MecDriveTrain mecDriveTrain;
  private DriveMec driveMec;
  private DriveMecTrackTarget driveMecTrackTarget;


  /*private Shooter shooter;
  private Shoot shoot;*/

  private Limelight limelight;

  private NavX navX;

  private PneumaticsTest pneumaticsTest;

  private TouchSensor touchSensor;

  private Move move;

  //public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> distanceVelocityMap;
  public LinearInterpolator distanceVelocityMap;
  private double[][] distanceVelocityData = { 
    {1.0, 10.0}, //{distance, velocity} format
    {3.0, 11.0}, 
    {10, 13.0} };

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverController = new XboxController(Constants.driverControllerPort);
    //operatorController = new XboxController(Constants.operatorControllerPort);

    //driveTrain = DriveTrain.getInstance();
    //driveGTA = new DriveGTA();
    //driveGTA.addRequirements(driveTrain);
    //driveTrain.setDefaultCommand(driveGTA);

    mecDriveTrain = new MecDriveTrain();
    driveMec = new DriveMec(mecDriveTrain);
    driveMec.addRequirements(mecDriveTrain);
    //driveMecTrackTarget = new DriveMecTrackTarget(mecDriveTrain);
    //driveMecTrackTarget.addRequirements(mecDriveTrain);
    mecDriveTrain.setDefaultCommand(driveMec);
    //mecDriveTrain.setDefaultCommand(driveMecTrackTarget);

    /*shooter = new Shooter();
    shoot = new Shoot(shooter);
    shoot.addRequirements(shooter);*/
    //shooter.setDefaultCommand(trackTarget);


    //limelight = new Limelight();

    navX = new NavX();

    //pneumaticsTest = new PneumaticsTest();

    //touchSensor = new TouchSensor();

    //move = new Move(driveTrain);
    //move.addRequirements(driveTrain);


    //distanceVelocityMap = new InterpolatingTreeMap<>(100);
    distanceVelocityMap = new LinearInterpolator(distanceVelocityData);

    //when distance is 10 m, velocity should be 16 m/s ??
    //TODO give more values and test output with smart dashboard
    //distanceVelocityMap.put(new IntesrpolatingDouble(10.), new InterpolatingDouble(16.)); 
    //distanceVelocityMap.put(new InterpolatingDouble(11.), new InterpolatingDouble(16.5)); 

    // Configure the button bindings
    configureButtonBindings();
  }



  public double getDriverRawAxis(final int axis){
    //try to make it easier to adjust using that fancy stuff
    try {
      return driverController.getRawAxis(axis);
    }
    catch(final RuntimeException exception) {
      DriverStation.reportError("Error getting raw axis because: " + exception.getMessage(), true);
    }
    //this error might have something to do with the squared values in DriveGTA
    return 0;
  }

  public double getDriverDeadzoneAxis(final int axis){
    try {
    final double rawValue = driverController.getRawAxis(axis);
    return (Math.abs(rawValue) <= Constants.deadzone) ? 0.0 : rawValue;
    }
    catch(final RuntimeException exception) {
      DriverStation.reportError("Error getting raw axis or returning deadzone axis because: " + exception.getMessage(), true);
    }
    return 0;
  }
  /*
  public double getOperatorDeadzoneAxis(int axis){
    double rawValue = operatorController.getRawAxis(axis);
    return Math.abs(rawValue) < Constants.deadzone ? 0.0 : rawValue;
  }
  */

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //final JoystickButton shootButton = new JoystickButton(driverController, Constants.shootButton);
    //shootButton.whileHeld(new Shoot(shooter)); 
    
    //final JoystickButton LLDistanceButton = new JoystickButton(driverController, Constants.LLDistanceButton);
    //LLDistanceButton.whileHeld(Limelight::getTargetDistance);

    final JoystickButton zeroYawButton = new JoystickButton(driverController, Constants.zeroYawButton);
    zeroYawButton.whenPressed(NavX::zeroGyroYaw); //this is a method reference 

    final JoystickButton zeroEncButton = new JoystickButton(driverController, Constants.zeroEncButton);
    zeroEncButton.whenPressed(mecDriveTrain::resetEncoders);

    //final JoystickButton driveNormalButton = new JoystickButton(driverController, Constants.driveNormalButton);
    //driveNormalButton.whileHeld(new DriveMec(mecDriveTrain));

    /*final JoystickButton PneumForwardButton = new JoystickButton(driverController, Constants.PneumForwardButton);
    PneumForwardButton.whileHeld(pneumaticsTest::setForward);

    final JoystickButton PneumReverseButton = new JoystickButton(driverController, Constants.PneumReverseButton);
    PneumReverseButton.whileHeld(pneumaticsTest::setReverse);*/

    //final JoystickButton PneumToggleButton = new JoystickButton(driverController, Constants.PneumToggleButton);
    //PneumToggleButton.whenPressed(pneumaticsTest::toggle);

    //final JoystickButton missTargetButton = new JoystickButton(driverController, Constants.missTargetButton);
    //missTargetButton.whileHeld(new MissTarget(mecDriveTrain, turret, hood, shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return getMecControllerCommand();
  }

  public Command getMecControllerCommand() {
    // Create config for trajectory
    /*TrajectoryConfig config =
        new TrajectoryConfig(Constants.maxMecSpeed,Constants.maxMecAcceleration)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(mecDriveTrain.getMecKinetimatics());
          
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);*/

    PPMecanumControllerCommand mecanumControllerCommand = new PPMecanumControllerCommand(
        Robot.testPath,
        mecDriveTrain::getPose,
        //TODO go to WPILIB source code for mecControlCommand and see how they use this feedforward
        //and then use that in the mecDriveTrain.setSpeeds() method
        //mecDriveTrain.getMecFeedforward(),
        mecDriveTrain.getMecKinematics(),

        // Position contollers
        new PIDController(Constants.kpMecXController, 0, 0),
        new PIDController(Constants.kpMecYController, 0, 0),
        new ProfiledPIDController(
            Constants.kpMecThetaController, 0, 0, Constants.mecThetaControllerConstraints),

        // Needed for normalizing wheel speeds
        //Constants.maxMecSpeed,

        // Velocity PID's
        /*new PIDController(Constants.kpMecL1Velocity, 0, 0),
        new PIDController(Constants.kpMecL2Velocity, 0, 0),
        new PIDController(Constants.kpMecR1Velocity, 0, 0),
        new PIDController(Constants.kpMecR2Velocity, 0, 0),*/
        //mecDriveTrain::getCurrentWheelSpeeds,
        mecDriveTrain::setSpeeds, // Consumer for the output motor voltages
        mecDriveTrain);

    return mecanumControllerCommand.andThen(mecDriveTrain::stopDrive);
  }
}
