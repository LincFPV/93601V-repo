# **93601V Repo**
## **What this is**
This serves as the primary place for coordinating code writing and other files for the team, will be periodically updated with the newest code or whenever I can be bothered to
## **What can you find here**
- drivetrain.v5cpp  --- our primary base code for our robot, runs the drive train, the claw and the (v1) lift (also new V2 lift)
- auton.v5cpp --- pretty self explanatory now in dev
- comp.v5cpp --- combine auton/drive code that works with the field controllers and is actually needed for the comp

## **Auton Logic**

After much brain storming I have come up with an achievable plan for an auton consisting of these steps

Toggle the toggle twice to match the colour of the team we are --> drive forward 2 feet --> rotate until Ai vision sensor sees target goal April tag at 0 degrees --> Move forward till bumper is triggered  --> move the pin cup preload into place.

## **Current robot: tank drive (branch `x-drive`)**
The robot was an X-drive; it is now a 44W tank drive (4 x 11W, two motors per side). The X-drive code is still in this branch's history.
- TankDrive.v5cpp --- driver code: arcade drive (left 10 + 9, right 20 + 19), 22W cascade lift (1, 2), pneumatic claw (3-wire A) and two side toggles on 5.5W motors (left 3, right 4, hold R2). TankDrive.cpp is the same code as plain C++
- OverrideComp.v5cpp --- full Override competition code for the tank drive: brain-screen auton selector (alliance, start wall, routine), odometry, six 15 second match autons, a 60 second skills auton and a drive test. OverrideComp.cpp is the same code as plain C++. Not tested on the robot yet, see the tuning steps at the top of the file
- ROBOT.md --- older handoff notes from the X-drive (ports, controls, open problems); the top of the file lists what has changed since
