// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.kauailabs.navx.AHRSProtocol.MagCalData;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FeedTimed;
import frc.robot.commands.SuckBalls;
import frc.robot.commands.TrackTargetCenterPose;
import frc.robot.commands.TrackTargetHood;
import frc.robot.commands.TrackTargetShooter;
import frc.robot.commands.TrackTargetTurret;
import frc.robot.library.Utilities;
import frc.robot.subsystems.Feeders;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MecDriveTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Still3Ball extends ParallelCommandGroup {
  PathPlannerTrajectory path1 = PathPlanner.loadPath("Still2BallRight1", 1, 2.5);
  PathPlannerTrajectory path2 = PathPlanner.loadPath("Still3Ball2", 1, 2);

  MecDriveTrain mecDriveTrain;
  Intake intake;
  Feeders feeders;
  Shooter shooter;
  Turret turret;
  Hood hood;

  PathPlannerState initialState = path1.getInitialState();

  /** Creates a new Still2Ball. */
  public Still3Ball(MecDriveTrain mecDriveTrain, Intake intake, Feeders feeders, Shooter shooter, Turret turret, Hood hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mecDriveTrain = mecDriveTrain;
    this.intake = intake;
    this.feeders = feeders;
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;

    //addRequirements(mecDriveTrain, intake, feeders, shooter, turret, hood);

    addCommands(
      sequence(
        new InstantCommand(() -> {
          NavX.zeroGyroYaw();
          NavX.setAngleOffset(initialState.holonomicRotation.getDegrees());
          RobotState.setPose(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
          //SmartDashboard.putNumber("HERE", 1);
        }),
        new WaitCommand(0.1), //why not
        
        race(
          sequence(
            Utilities.getMecControllerCommand(path1, mecDriveTrain).andThen(mecDriveTrain::stopDrive),
            new WaitCommand(1), //gives time for shooter to get the right speed
            new FeedTimed(feeders, 3)
          ),
          new TrackTargetTurret(false, turret) //LL doesnt work if too close
        ),

        parallel(
          sequence(
            Utilities.getMecControllerCommand(path2, mecDriveTrain).andThen(mecDriveTrain::stopDrive),
            new WaitCommand(1), //gives time for shooter to get the right speed
            new FeedTimed(feeders, 4)
          ),
          new TrackTargetTurret(true, turret) 
        )
      ),
      new SuckBalls(intake),
      new TrackTargetHood(hood),
      new TrackTargetShooter(shooter)
      //new TrackTargetTurret(false, turret)    
    );
  }


}
