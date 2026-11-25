# **93601V Repo**

> **Switching computers or new to the repo? Read [HANDOFF.md](HANDOFF.md).** Get the latest with `./scripts/sync.sh`, save your work with `./scripts/sync.sh "what you changed"`.
## **What this is**
This serves as the primary place for coordinating code writing and other files for the team, will be periodically updated with the newest code or whenever I can be bothered to
## **What can you find here**
- drivetrain.v5cpp  --- our primary base code for our robot, runs the drive train, the claw and the (v1) lift (also new V2 lift)
- auton.v5cpp --- pretty self explanatory now in dev
- comp.v5cpp --- combine auton/drive code that works with the field controllers and is actually needed for the comp

## **Auton Logic**

After much brain storming I have come up with an achievable plan for an auton consisting of these steps

Toggle the toggle twice to match the colour of the team we are --> drive forward 2 feet --> rotate until Ai vision sensor sees target goal April tag at 0 degrees --> Move forward till bumper is triggered  --> move the pin cup preload into place.

## **X-drive and tank drive code (branch `x-drive`)**
Both drive bases are kept. Use the pair that matches the robot you have built. The claw (3-wire A) and controls (hold R2 for the side toggles) are the same in both, but the ports are not.

**Tank drive (44W, two 11W motors per side) -- the current robot**

| Device | Port |
|---|---|
| Front left / back left drive | 3 / 11 |
| Front right / back right drive | 8 / 20 |
| Inertial | 16 |
| Lift (2 x 11W) | 1 and 10 |
| Side toggles (5.5W) | left 2, right 9 |
| Claw solenoid | 3-wire A |

Drive, lift and side toggles run on voltage (12 V = full power) for more speed.
- TankDrive.v5cpp --- driver code: arcade drive (left stick forward/back, right stick turn)
- TankComp.v5cpp --- full Override competition code: brain-screen auton selector, odometry, six 15 second match autons, a 60 second skills auton and a drive test

**X-drive (44W, one 11W motor per wheel: FL 10, FR 20, BL 9, BR 19, inertial 12, lift 1 + 2, side toggles left 3, right 4)** -- still on the old ports
- XDrive.v5cpp --- driver code with field-centric driving and strafing
- XDriveComp.v5cpp --- full Override competition code for the X-drive, same routines as TankComp

Each .v5cpp has a matching .cpp with the same code as plain C++. None of the competition code has been tested on a robot yet; see the tuning steps at the top of each file.
- ROBOT.md --- handoff notes (ports, controls, open problems); the top of the file lists what has changed since
