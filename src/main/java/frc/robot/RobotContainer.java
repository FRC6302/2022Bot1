// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DistanceToTarget;
import frc.robot.commands.DriveGTA;
import frc.robot.commands.DriveMec;
import frc.robot.commands.Move;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.PneumaticsTest;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
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

  /*private DriveTrain driveTrain;
  //private DriveGTA driveGTA;

  private MecDriveTrain mecDriveTrain;
  private DriveMec driveMec;*/

  private Shooter shooter;
  private Shoot shoot;

  private Limelight limelight;
  private DistanceToTarget distanceToTarget;

  private NavX navX;

  private PneumaticsTest pneumaticsTest;

  private Move move;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverController = new XboxController(Constants.driverControllerPort);
    //operatorController = new XboxController(Constants.operatorControllerPort);

    //driveTrain = DriveTrain.getInstance();
    //driveGTA = new DriveGTA();
    //driveGTA.addRequirements(driveTrain);
    //driveTrain.setDefaultCommand(driveGTA);

    /*mecDriveTrain = new MecDriveTrain();
    driveMec = new DriveMec(mecDriveTrain);
    driveMec.addRequirements(mecDriveTrain);
    mecDriveTrain.setDefaultCommand(driveMec);*/

    shooter = new Shooter();
    shoot = new Shoot(shooter);
    shoot.addRequirements(shooter);


    //limelight = new Limelight();
    //distanceToTarget = new DistanceToTarget();
    //distanceToTarget.addRequirements(limelight);

    navX = new NavX();

    //pneumaticsTest = new PneumaticsTest();

    //move = new Move(driveTrain);
    //move.addRequirements(driveTrain);


    // Configure the button bindings
    configureButtonBindings();
  }



  public double getDriverRawAxis(final int axis){
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
    final JoystickButton shootButton = new JoystickButton(driverController, Constants.shootButton);
    shootButton.whileHeld(shoot); 
    
    //final JoystickButton LLDistanceButton = new JoystickButton(driverController, Constants.LLDistanceButton);
    //LLDistanceButton.whileHeld(new DistanceToTarget());

    //final JoystickButton zeroYawButton = new JoystickButton(driverController, Constants.zeroYawButton);
    //zeroYawButton.whenPressed(NavX::zeroGyroYaw); //this is a method reference 

    /*final JoystickButton PneumForwardButton = new JoystickButton(driverController, Constants.PneumForwardButton);
    PneumForwardButton.whileHeld(pneumaticsTest::setForward);

    final JoystickButton PneumReverseButton = new JoystickButton(driverController, Constants.PneumReverseButton);
    PneumReverseButton.whileHeld(pneumaticsTest::setReverse);*/

    //final JoystickButton PneumToggleButton = new JoystickButton(driverController, Constants.PneumToggleButton);
    //PneumToggleButton.whenPressed(pneumaticsTest::toggle);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return move;
  }
}
