# **93601V Repo**

> **Switching computers or new to the repo? Read [HANDOFF.md](HANDOFF.md).** Get the latest with `./scripts/sync.sh`, save your work with `./scripts/sync.sh "what you changed"`.

Code for team 93601V's VEX V5 robot, game: Override (2026-27).

## The current robot: tank drive (44W, two 11W motors per side)

| File | What it is |
|---|---|
| `TankComp.v5cpp` | **Flash this for matches and skills.** Driver control plus a brain-screen auton selector, odometry, six 15 second match autons, a 60 second skills auton and a drive test. |
| `TankDrive.v5cpp` | Driver control only, for practice. |

The repo folder is also a VEX VS Code project that builds and downloads `TankComp` (see `HANDOFF.md`).

Each `.v5cpp` (open in VEXcode V5) has a matching `.cpp` with the same code as plain C++ (edit in VS Code or read on GitHub). `scripts/sync.sh` keeps them matching.

| Device | Port |
|---|---|
| Front left / back left drive | 3 / 11 |
| Front right / back right drive | 8 / 20 (reversed) |
| Inertial | 16 |
| Lift (2 x 11W) | 1 and 10 |
| Side toggles (5.5W) | left 2, right 9 |
| Claw solenoid | 3-wire A |

Drive, lift and side toggles run on voltage (12 V = full power). Controls: left stick forward/back, right stick turn, R1/R2 lift up/down, L1 claw close, L2 claw open, hold A for the side toggles (hold Up to reverse them). For practice only, B switches to tank controls (left stick = left side, right stick = right side). While you drive in arcade with the turn stick centred, a drive-straight assist uses the inertial to stop the robot drifting to one side (`DRIVE_STRAIGHT_ASSIST`).

None of the competition code has been tested on the robot yet; see the tuning steps at the top of `TankComp.cpp` and the to-do list in `HANDOFF.md`.

## Old code

Code we no longer use is on the [`deprecated` branch](https://github.com/LincFPV/93601V-repo/tree/deprecated):
- `old-robot/`: the earlier tank robot with the spin claw (`drivetrain`, `auton`, `Comp`, `CompV2`, `OldDrivetrainComp`) and its original README
- `x-drive/`: the X-drive version of this robot (`XDrive`, `XDriveComp`) and its handoff notes (`ROBOT.md`)
