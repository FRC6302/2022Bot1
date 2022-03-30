// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveMec;
import frc.robot.commands.FeedBoth;
import frc.robot.commands.SuckBalls;
import frc.robot.commands.TrackTargetCenterPose;
import frc.robot.commands.TrackTargetLeadingPose;
import frc.robot.commands.TurnTurret;
import frc.robot.commands.Auto.Still5Ball;
import frc.robot.library.Data;
import frc.robot.subsystems.Feeders;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightGoal;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private XboxController operatorController;

  //private DriveTrain driveTrain;
  //private DriveGTA driveGTA;

  private MecDriveTrain mecDriveTrain;
  private DriveMec driveMec;
  //private DriveMecTrackTarget driveMecTrackTarget;


  private Shooter shooter;
  //private Shoot shoot;

  private Turret turret;
  //private TurnTurret turnTurret;
  //private TrackTargetCenter trackTargetCenter;
  //private TrackTargetCenterPose trackTargetCenterPose;

  private Hood hood;
  //private RaiseHood raiseHood;

  private Intake intake;
  //private SuckBalls suckBalls;

  //private Climbers climbers;
  //private MoveClimbers moveClimbers;

  //private FeederFront feederFront;
  //private FeederMiddle feederMiddle;
  private Feeders feeders;
  //private FeedBoth feedBoth;
  //private FeedColorBased feedColorBased;

  private LimelightGoal limelight;

  private NavX navX;

  //private ColorSensor colorSensor;

  //private LEDs leds;

  //private PneumaticsTest pneumaticsTest;

  //private TouchSensor touchSensor;

  //private Move move;

  //private NeoTest neoTest;
  //private TestNeo testNeo;

  private Data data;
  private RobotState robotState;

  Still5Ball still5Ball;
  
  private SendableChooser<String> autonChooser;
  private final String moveForward = "move forward";
  private final String sixBall = "6 ball";

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverController = new XboxController(Constants.driverControllerPort);
    operatorController = new XboxController(Constants.operatorControllerPort);

    mecDriveTrain = new MecDriveTrain();
    driveMec = new DriveMec(mecDriveTrain);
    driveMec.addRequirements(mecDriveTrain);
    //driveMecTrackTarget = new DriveMecTrackTarget(mecDriveTrain);
    //driveMecTrackTarget.addRequirements(mecDriveTrain);
    mecDriveTrain.setDefaultCommand(driveMec);
    
    shooter = new Shooter();
    //shoot = new Shoot(shooter);
    //shoot.addRequirements(shooter);
    //shooter.setDefaultCommand(trackTarget);

    turret = new Turret();
    //turnTurret = new TurnTurret(turret);
    //turnTurret.addRequirements(turret);
    //turret.setDefaultCommand(turnTurret);
    //trackTargetCenterPose = new TrackTargetCenterPose(mecDriveTrain, turret, shooter);
    //turret.setDefaultCommand(trackTargetCenterPose);

    hood = new Hood();
    //raiseHood = new RaiseHood(hood);
    //raiseHood.addRequirements(hood);
    //hood.setDefaultCommand(raiseHood);

    intake = new Intake();
    //suckBalls = new SuckBalls(intake);
    //suckBalls.addRequirements(intake);

    feeders = new Feeders();
    //feedBoth = new FeedBoth(feeders);
    //feedColorBased = new FeedColorBased(feeders);
    //feeders.setDefaultCommand(feedColorBased);

    //climbers = new Climbers();
    //moveClimbers = new MoveClimbers(climbers);


    limelight = new LimelightGoal();

    navX = new NavX();

    //colorSensor = new ColorSensor();

    //leds = new LEDs();

    //pneumaticsTest = new PneumaticsTest();

    //touchSensor = new TouchSensor();

    //move = new Move(driveTrain);
    //move.addRequirements(driveTrain);

    //neoTest = new NeoTest();
    //testNeo = new TestNeo(neoTest);
    //testNeo.addRequirements(neoTest);
    //neoTest.setDefaultCommand(testNeo); 


    data = new Data();
    robotState = new RobotState(mecDriveTrain);

    //when distance is 10 m, velocity should be 16 m/s ??
    //TODO give more values and test output with smart dashboard
    //distanceVelocityMap.put(new InterpolatingDouble(10.), new InterpolatingDouble(16.)); 
    //distanceVelocityMap.put(new InterpolatingDouble(11.), new InterpolatingDouble(16.5)); 

    still5Ball = new Still5Ball(mecDriveTrain, intake, feeders, shooter, turret, hood);

    autonChooser = new SendableChooser<>();
    autonChooser.setDefaultOption(moveForward, moveForward);
    autonChooser.addOption(sixBall, sixBall);

    // Configure the button bindings
    configureButtonBindings();
  }



  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //driverController.setRumble(RumbleType.kLeftRumble, 0.5);

    //final JoystickButton driveButton = new JoystickButton(driverController, Constants.driveButton);
    //driveButton.whileHeld(new DriveMec(mecDriveTrain));
    
    /*final JoystickButton shootButton = new JoystickButton(driverController, Constants.shootButton);
    /*shootButton.whileHeld(() -> {
      shooter.setMotors(.99);
    });
    shootButton.whenReleased(() -> {
      shooter.stop();
    });*/
    //shootButton.whileHeld(new Shoot(shooter, 0.1, 0.1));*/

    //final JoystickButton shootButton2 = new JoystickButton(driverController, Constants.shootButton2);
    //shootButton2.whileHeld(new Shoot(shooter, 0.16, 0.16));  

    final JoystickButton turnTurretButton = new JoystickButton(driverController, Constants.turnTurretButton);
    turnTurretButton.whileHeld(() -> {
      //turret.setMotorPosPID(Limelight.getLastX(), 0, 2, 0);
      turret.setMotor(getDriverRawAxis(Constants.rightStickX) / -4);
    });
    turnTurretButton.whenReleased(() -> {
      turret.stopMotor();
    });

    //final JoystickButton trackButton = new JoystickButton(driverController, Constants.trackButton);
    //trackButton.whileHeld(new TrackTargetCenterPose(mecDriveTrain, turret, hood, shooter));

    //final JoystickButton turnTurret2Button = new JoystickButton(driverController, Constants.turnTurret2Button);
    //turnTurret2Button.whileHeld(new TurnTurret(turret));

    //final JoystickButton raiseHoodButton = new JoystickButton(driverController, Constants.raiseHoodButton);
    //raiseHoodButton.whileHeld(new RaiseHood(hood));

    final JoystickButton raiseHood2Button = new JoystickButton(driverController, Constants.raiseHood2Button);
    raiseHood2Button.whileHeld(() -> {
      hood.setMotor(getDriverDeadzoneAxis(Constants.rightStickY) / -4);
    });
    raiseHood2Button.whenReleased(() -> {
      hood.stopMotor();
    });

    //final JoystickButton intakeButton = new JoystickButton(driverController, Constants.intakeButton); 
    //intakeButton.whileHeld(new SuckBalls(intake));

    /*final JoystickButton feedBothButton = new JoystickButton(driverController, Constants.feedBothButton);
    feedBothButton.whileHeld(new FeedBoth(feeders));
    //feedBothButton.whileHeld(new FeedColorBased(feeders));
    feedBothButton.whenReleased(() -> {
      feeders.stopBothMotors();
    });*/

    //final JoystickButton moveClimbersButton = new JoystickButton(driverController, Constants.moveClimbersButton);
    //moveClimbersButton.whenHeld(new MoveClimbers(climbers));

    final JoystickButton closeToBallsButton = new JoystickButton(driverController, Constants.closeToBallsButton);
    closeToBallsButton.whileHeld(new ParallelCommandGroup(
      new FeedBoth(feeders),
      new SuckBalls(intake),
      //new TrackTargetCenterPose(mecDriveTrain, turret, hood, shooter)
      new TrackTargetLeadingPose(mecDriveTrain, turret, hood, shooter)
      //new Shoot(shooter)
    ));
    closeToBallsButton.whenReleased(() -> {
      feeders.stopBothMotors();
      intake.stopMotor();
      turret.stopMotor();
      hood.stopMotor();
      shooter.stopMotors();
    });
    

    /*final JoystickButton farAwayButton = new JoystickButton(driverController, Constants.farAwayButton);
    farAwayButton.whileHeld(new ParallelCommandGroup(
      new InstantCommand(() -> {
        feeders.stopBothMotors();
      }, 
      feeders)
    ));

    farAwayButton.and(closeToBallsButton).whileActiveContinuous(new ParallelCommandGroup(
      new SuckBalls(intake),
      new InstantCommand(() -> {
        feeders.stopBothMotors();
      }, 
      feeders)
    ));*/



    final JoystickButton zeroTurretButton = new JoystickButton(driverController, Constants.zeroTurretButton);
    zeroTurretButton.whenPressed(turret::resetEncoder);

    //final JoystickButton zeroHoodButton = new JoystickButton(driverController, Constants.zeroHoodButton);
    //zeroHoodButton.whenPressed(hood::resetEncoder);

    
    //final JoystickButton LLDistanceButton = new JoystickButton(driverController, Constants.LLDistanceButton);
    //LLDistanceButton.whileHeld(Limelight::getTargetDistance);

    //final JoystickButton zeroYawButton = new JoystickButton(driverController, Constants.zeroYawButton);
    //zeroYawButton.whenPressed(NavX::zeroGyroYaw); //this is a method reference 

    //final JoystickButton zeroEncButton = new JoystickButton(driverController, Constants.zeroEncButton);
    //zeroEncButton.whenPressed(mecDriveTrain::resetEncoders);

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
    //return null;
    String pathStr = autonChooser.getSelected();
    PathPlannerTrajectory trajectory = null;

    /*if (pathStr == null) {
      return new InstantCommand();
    }
    else {
      if (pathStr == moveForward) {
        trajectory = Robot.moveForwardPath;
        //return getMecControllerCommand(trajectory);
      }
      else if (pathStr == sixBall) {
        trajectory = Robot.sixBallPath;

      }
    }*/

    switch (pathStr) {
      case moveForward:  
        trajectory = Robot.moveForwardPath;
        //return getMecControllerCommand(trajectory).alongWith(parallel);
        break;
      case sixBall:
        trajectory = Robot.sixBallPath;
        break;
      default:
        return new InstantCommand();
    }

    //return getMecControllerCommand(trajectory);
    return still5Ball;

    // An ExampleCommand will run in autonomous
    //return getMecControllerCommand();
  }

  /*public Command getMecControllerCommand(PathPlannerTrajectory trajectory) {
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
    
    /*ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kpMecThetaController, 0., 0., 
      new Constraints(Constants.maxMecRotationVelocity, Constants.maxMecRotationAccel));
      
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.reset(mecDriveTrain.getPoseEstimate().getRotation().getDegrees());

    mecDriveTrain.setPose(Robot.testPath.getInitialPose());

    PPMecanumControllerCommand mecanumControllerCommand = new PPMecanumControllerCommand(
        trajectory,
        mecDriveTrain::getPoseEstimate,
        //TODO go to WPILIB source code for mecControlCommand and see how they use this feedforward
        //and then use that in the mecDriveTrain.setSpeeds() method
        //mecDriveTrain.getMecFeedforward(),
        mecDriveTrain.getMecKinematics(),

        // Position contollers
        new PIDController(Constants.kpMecXController, 0, 0),
        new PIDController(Constants.kpMecYController, 0, 0),
        thetaController,

        // Needed for normalizing wheel speeds
        //Constants.maxMecSpeed,

        // Velocity PID's
        /*new PIDController(Constants.kpMecL1Velocity, 0, 0),
        new PIDController(Constants.kpMecL2Velocity, 0, 0),
        new PIDController(Constants.kpMecR1Velocity, 0, 0),
        new PIDController(Constants.kpMecR2Velocity, 0, 0),
        //mecDriveTrain::getCurrentWheelSpeeds,
        //mecDriveTrain::setSpeeds, // Consumer for the output motor voltages
        //mecDriveTrain);

    return new InstantCommand(() -> mecDriveTrain.setPose(Robot.testPath.getInitialPose()))
    .andThen(mecanumControllerCommand)
    .andThen(mecDriveTrain::stopDrive);
  }

  public void updateOdometry() {
    mecDriveTrain.updateOdometry();
  }*/
  
  
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
  
  public double getOperatorDeadzoneAxis(int axis){
    double rawValue = operatorController.getRawAxis(axis);
    return Math.abs(rawValue) < Constants.deadzone ? 0.0 : rawValue;
  }

  public JoystickButton getDriverButton(int buttonNumber) {
    return new JoystickButton(driverController, buttonNumber);
  }

  public JoystickButton getOperatorButton(int buttonNumber) {
    return new JoystickButton(operatorController, buttonNumber);
  }


  public void updateShooterVelocities() {
    shooter.updateVelocities();
  }
}
