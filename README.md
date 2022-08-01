# 2022Bot1
FRC Team 6302's robot code for the 2022 Rapid React season

## Robot functionality:
1. mecanum drive
2. 360ish degree turret
3. adjustable hood
4. two flywheels on hood
5. no climbers
6. sensors: external encoders (4 drive, 1 turret), limelight, NavX gyro

## How everything works
Using something called a mecanum pose estimator, the code combines data from the drive encoders and limelight to determine where the robot is on the field. The goal is always in the same place, so if it knows its own position, it knows where to point the turret so that it is always facing towards the target. 

## Most important/coolest files:
1. RobotState.java (my baby) keeps track of the robot pose and all the shooting while moving stuff
2. TrackTargetTurret.java takes pose data and figure out where to point the turret to track the target

## Notes:
1. I wrote descriptions for most files (that aren't self explanatory) at the top of them
2. originally I used TrackTargetCenterPose.java for controlling basically everything on the robot except the drive, but realized I needed more flexibility and the ability to only have some subsystems running but not others. So I split that file up into TrackTargetTurret.java, TrackTargetShooter.java, etc. 
3. on that note, there are a lot of unused files and code that is just leftover from all my trials and that I didn't want to delete, just in case I ever needed it again. It should all be either commented out or have a note saying that it is unused.
4. RobotState.java doesn't correspond to any actual hardware like most other subsystems, I just made it a subsystem so it would automatically update every loop with the periodic() method. There's probably a better way to do it but I like this one.

## Shooting while moving
This project has the framework and correct math for shooting while moving, but I never got around to making everything perfect enough for it to actually work. For it to work, I would need to:
1. make the turret feedforward and tracking more accurate
2. same for the shooter, need to be able to change shooting speed quickly and accurately
3. be able to score consistently from any distance while not moving (our biggest problem lol)
4. upgrade the pose estimator kalman filter to the better version
5. figure how to get more consistent limelight data
6. incorporate an acceleration factor to correct for robot acceleration while shooting


-Smashley
