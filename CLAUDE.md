# 93601V VEX V5 robot code

Team 93601V, VEX V5 Robotics Competition, game: Override (2026-27).

Read `HANDOFF.md` first ("Where we're up to" is the current state). The code
headers are the source of truth for ports.

## Files

- Current robot is a tank drive: `TankDrive` (driver only) and `TankComp`
  (competition: brain-screen auton selector, odometry, 15 s match autons,
  60 s skills, TEST routine).
- Old code (the X-drive version and the earlier robot, plus the old
  `ROBOT.md` notes) is on the `deprecated` branch, not on `main`.
- Every program exists as `Name.cpp` and `Name.v5cpp`. The `.v5cpp` is JSON with
  the code in `textContent`. After editing either, run
  `python3 scripts/v5cpp_sync.py` so they match. Never leave a pair out of sync.

## VEXcode rules (these have caused build errors before)

- Do not add `brain Brain;` (VEXcode declares it) and keep the Devices panel /
  `robotConfig` empty; all devices are declared in the code.
- The repo root is also a VEX VS Code project for TankComp: `src/main.cpp`
  defines `vex::brain Brain;` and then includes `../TankComp.cpp`. The brain
  must stay above the #include in that same file: in a separate .cpp it gets
  created after the claw (`Brain.ThreeWirePort.A`) and the brain crashes with a
  memory permission error. Edit `TankComp.cpp`, never `src/main.cpp`.
- Use `Brain.Screen.printAt(x, y, true, "...")` with the `true` argument
  (without it `printAt` is ambiguous).

## Auton driving (TankComp)

- Driver control uses `tankDrive()` (voltage). Autons use `autonDrive()`
  (closed-loop percent velocity, `DRIVE_SIGN` only). Don't mix them up.
- Heading comes from `fusedHeading` (inertial x `IMU_SCALE`, falling back to the
  wheels and `TRACK_WIDTH`). Tuning steps are in `HANDOFF.md`, "Making the
  auton accurate". A simulator run is not proof it works on the robot.

## Working here

- Students own this code. Explain changes in plain words, keep comments simple.
- Nothing is verified on the robot unless `HANDOFF.md` says so; say so too.
- Before stopping: add a dated line to "Where we're up to" in `HANDOFF.md`,
  then save with `./scripts/sync.sh "short message"` if the user wants it
  pushed. Ask before pushing to `main`.
