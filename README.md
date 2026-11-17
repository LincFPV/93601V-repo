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

## **X-Drive robot (branch `x-drive`)**
- XDrive.v5cpp --- X-drive driver code with field-centric driving (FL 10, FR 20, BL 9, BR 19, inertial 12, radio 11), 22W cascade lift (1, 2), pneumatic claw (3-wire A) and two side toggles on 5.5W motors (left 3, right 4). XDrive.cpp is the same code as plain C++
- OverrideComp.v5cpp --- full Override competition code for the X-drive: brain-screen auton selector (alliance, start wall, routine), odometry, six 15 second match autons, a 60 second skills auton and a drive test. OverrideComp.cpp is the same code as plain C++. Not tested on the robot yet, see the tuning steps at the top of the file
- ROBOT.md --- handoff notes: ports, controls, how the code works and open problems
