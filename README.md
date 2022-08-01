# 2022Bot1
FRC Team 6302's robot code for the 2022 Rapid React season

Robot functionality:
1. mecanum drive
2. 360ish degree turret
3. adjustable hood
4. two flywheels on hood
5. sensors: external encoders (4 drive, 1 turret), limelight, NavX gyro

Most important/coolest files:
1. RobotState.java (my baby)
2. TrackTargetTurret.java

Notes:
1. I wrote descriptions for most files (that aren't self explanatory) at the top of them
2. originally I used TrackTargetCenterPose.java for controlling basically everything on the robot except the drive, but realized I needed more flexibility and the ability to only have some subsystems running but not others. So I split that file up into TrackTargetTurret.java, TrackTargetShooter.java, etc. 
3. on that note, there are a lot of files and code that is just leftover from all my trials and I didn't want to delete just in case. I tried to leave a comment there if that was the case
4. RobotState.java doesn't correspond to any actual hardware like most other subsystems, I just made it a subsystem so it would automatically update every loop with the periodic() method. There's probably a better way to do it but I like this one.
5. 

This project has the framework and correct math for shooting while moving, but I never got around to making everything perfect enough for it to actually work. For it to work, I would need to:
1. make the turret feedforward and tracking more accurate
2. same for the shooter, need to be able to change shooting speed quickly and accurately
3. be able to score consistently from any distance while not moving (our biggest problem lol)
4. upgrade the pose estimator kalman filter to the better version
5. figure how to get more consistent limelight data
6. incorporate an acceleration factor to correct for robot acceleration while shooting


-Smashley
