# 93601V VEX V5 robot code

Team 93601V, VEX V5 Robotics Competition, game: Override (2026-27).

Read `HANDOFF.md` first ("Where we're up to" is the current state), then
`ROBOT.md` for older background. The code headers are the source of truth for
ports; the port tables in `ROBOT.md` are out of date.

## Files

- Current robot is a tank drive: `TankDrive` (driver only) and `TankComp`
  (competition: brain-screen auton selector, odometry, 15 s match autons,
  60 s skills, TEST routine).
- `XDrive` / `OverrideComp` (and `XDriveComp` on the `x-drive` branch) are the
  earlier X-drive version. `drivetrain`, `auton`, `Comp`, `CompV2`,
  `OldDrivetrainComp` are an older robot.
- Every program exists as `Name.cpp` and `Name.v5cpp`. The `.v5cpp` is JSON with
  the code in `textContent`. After editing either, run
  `python3 scripts/v5cpp_sync.py` so they match. Never leave a pair out of sync.

## VEXcode rules (these have caused build errors before)

- Do not add `brain Brain;` (VEXcode declares it) and keep the Devices panel /
  `robotConfig` empty; all devices are declared in the code.
- Use `Brain.Screen.printAt(x, y, true, "...")` with the `true` argument
  (without it `printAt` is ambiguous).

## Working here

- Students own this code. Explain changes in plain words, keep comments simple.
- Nothing is verified on the robot unless `HANDOFF.md` says so; say so too.
- Before stopping: add a dated line to "Where we're up to" in `HANDOFF.md`,
  then save with `./scripts/sync.sh "short message"` if the user wants it
  pushed. Ask before pushing to `main`.
