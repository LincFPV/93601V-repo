# Copilot instructions: 93601V VEX V5 robot code

Team 93601V, VEX V5 Robotics Competition, game **Override** (2026-27).
Students own this code: explain changes in plain words and keep comments simple.
Nothing here has been tested on the robot unless a note says so.

## The current robot (tank drive)

- **`TankDrive.cpp`** (driver only) and **`TankComp.cpp`** (competition: brain-screen
  auton selector, odometry, six 15 s match autons, a 60 s skills auton, TEST routine).
- Ports: drive left 3 (front) + 11 (back), right 8 (front) + 20 (back, right side
  reversed); inertial 16; lift 1 + 10 (2 x 11W); side toggles left 2, right 9
  (5.5W motors); claw solenoid 3-wire A. The code headers are the source of truth.
- Drive geared 36:60 (motor:wheel). Drive, lift and toggles run on voltage
  (`spin(..., volt)`, 100% = 12 V via `VOLTS_PER_PERCENT`).
- Controls: left stick forward/back, right stick turn; L1/L2 lift; R1 claw;
  hold R2 = both side toggles spin, hold Up = they spin the other way.

## Other files

- `XDrive.cpp` / `OverrideComp.cpp`: an older X-drive version (single toggle motor
  and colour sensor). The newest X-drive files (`XDrive`, `XDriveComp`) are on the
  `x-drive` branch. `drivetrain`, `auton`, `Comp`, `CompV2`, `OldDrivetrainComp` are
  an older robot.
- Every program is stored twice: `Name.cpp` and `Name.v5cpp` (VEXcode V5 project,
  JSON with the code in `"textContent"`). **After editing a `.cpp`, the matching
  `.v5cpp` must get the same code**, or VEXcode downloads the old version.

## VEXcode rules (these caused build errors before)

- Never add `brain Brain;` (VEXcode declares it). Keep the VEXcode Devices panel /
  `robotConfig` empty: every device is declared in the code.
- Use `Brain.Screen.printAt(x, y, true, "...")` with the `true` argument.

## What has been done (newest first)

- **26 Nov 2026 (not committed yet, branch `slower-toggle`):** side toggle speed
  lowered from 100% to 50% (`TOGGLE_SPEED` in `TankDrive.cpp` and `TankComp.cpp`,
  `.v5cpp` files updated). Also slows the auton toggle flip.
- **25 Nov 2026:** Pull request #2 (branch `handoff-tools`, not merged yet) adds
  `HANDOFF.md`, `CLAUDE.md`, `scripts/sync.sh` (match .cpp/.v5cpp, commit, pull,
  push), `scripts/v5cpp_sync.py` and `scripts/setup-mac.sh`.
- **24 Nov 2026:** tank drive moved to the ports above and to voltage control;
  auton minimum powers raised (`MIN_MOVE` 20, `MIN_TURN` 15).
- **19 Nov 2026:** robot changed from X-drive to a 44W tank drive; autons rewritten
  as turn-then-drive moves with encoder + inertial odometry.
- Earlier: Override autons written, side toggles replaced the single toggle motor,
  lift cut from 3 to 2 motors, 36:60 drive gearing added.

## Still to do

- Check `DRIVE_SIGN` / `TURN_SIGN` on the robot, then run the TEST routine (X + Y).
- Measure wheel diameter, claw reach, lift heights (`H_...`), `TOGGLE_FLIP_DEGREES`.
- Tune the autons one at a time; see the tuning steps at the top of `TankComp.cpp`.
