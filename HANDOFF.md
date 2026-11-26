# Switching devices: start here

This repo is how the team moves work between computers. Everything you need is
in GitHub, so any computer can pick up where the last one stopped. No zip files,
no copying code by hand.

## Every time

**When you sit down at a computer**, get the latest code first:

```bash
./scripts/sync.sh
```

**When you finish**, update "Where we're up to" below, then save everything to
GitHub with a short note about what you did:

```bash
./scripts/sync.sh "tuned TankComp turn speed"
```

`sync.sh` does four things: makes each `.cpp` match its `.v5cpp`, commits your
changes, pulls everyone else's on top, and pushes. If two people changed the
same lines it stops and tells you what to do; nothing gets lost.

Run these from the repo folder: in VS Code, open **Terminal → New Terminal**.

## First time on a new computer

**Mac:** open Terminal and paste this one line. It installs git, the GitHub
tool, VS Code and the VEX extension, signs you in to GitHub, and puts the repo
in `~/Robotics/93601V-repo`. It is safe to run again later to update everything.

```bash
curl -fsSL https://raw.githubusercontent.com/LincFPV/93601V-repo/main/scripts/setup-mac.sh | bash
```

**Windows:**

1. Install [Git for Windows](https://git-scm.com/download/win) (pick the
   defaults; it includes *Git Bash*), [Python](https://www.python.org/downloads/)
   (tick "Add python.exe to PATH") and [VS Code](https://code.visualstudio.com/).
2. In VS Code, install the **VEX Robotics** extension (VS Code also offers it
   when you open the repo, because `.vscode/extensions.json` recommends it).
3. In VS Code press **Ctrl+Shift+P → Git: Clone**, paste
   `https://github.com/LincFPV/93601V-repo.git`, and sign in to GitHub when asked.
4. Use the sync script from a *Git Bash* terminal (VS Code: Terminal → New
   Terminal → the **+** dropdown → Git Bash).

## Editing the code

Every program is kept twice: `Name.cpp` (plain C++, edit this in VS Code or read
it on GitHub) and `Name.v5cpp` (the same code as a VEXcode V5 project, open this
in VEXcode to download to the robot).

- Edit either one. `sync.sh` copies your change into the other.
- To check them without saving: `python3 scripts/v5cpp_sync.py --check`

### Downloading to the robot

**From VS Code (TankComp):** the repo folder is a VEX VS Code project. Open the
repo folder in VS Code, plug in the brain or controller, then use the VEX
buttons at the bottom of the window (or the VEX icon in the left bar): **Build**,
then **Download**. It goes to slot 1 as "TankComp".
- Edit `TankComp.cpp` in the main folder. `src/main.cpp` creates the brain and
  then includes it (VEXcode does the same by itself), so VS Code and VEXcode
  always build exactly the same program.
- The first time, the VEX extension may download its SDK and compiler; let it.

**From VEXcode V5:** open `TankComp.v5cpp` (or `TankDrive.v5cpp` for driver
practice only) and download as usual.
- In VEXcode, keep the **Devices panel empty** and never add `brain Brain;`.
  Every device is declared in the code; doing either gives "redefinition" errors.

## Making the auton accurate

Do these in order, with the robot on the field tiles and room to move. Pick
each routine on the brain screen and press **X + Y** on the controller (only
works when the controller isn't plugged into field control). Down arrow stops it.
Results show on the bottom two lines of the brain screen.

1. **Inertial:** the brain line `IMU` must say **OK**. *NOT FOUND* = plug the
   inertial into port 16. *moved?* = it was bumped while calibrating: keep the
   robot completely still and tap **CAL** (calibration takes about 3 seconds
   and also happens every time the program starts).
2. **CHECK ROBOT:** spins a little each way. It tells you if `DRIVE_SIGN` is
   wrong and shows the robot's `TRACK_WIDTH`. Put that number into the code.
3. **TUNE: DRIVE 48in:** drives straight. Measure how far it really went with a
   tape measure (D inches), then set
   `FORWARD_SCALE = FORWARD_SCALE x D / (the distance it shows)`.
   Repeat until it's within about half an inch.
4. **TUNE: TURN 90 x4:** four right-angle turns. It should end facing exactly
   where it started. If it's off, do the hand-turn test:
   - Line the robot up against a tile seam and read the `raw` number on the
     `IMU` line.
   - Turn the robot by hand **exactly 5 full turns clockwise** back onto the
     same seam, and read `raw` again.
   - `IMU_SCALE = 1800 / (second number - first number)`.
5. **TEST (24in square):** should come back to where it started.
6. Then the lift heights, claw reach and toggle, as listed at the top of
   `TankComp.cpp`.

If a routine ever shows **"STOPPED: robot turned the wrong way"**, the robot
stopped itself instead of spinning out of control: run CHECK ROBOT.

`DRIVE_DECEL` and `TURN_DECEL` set how hard the robot slows down at the end of
each move. Raise them a little for speed once it's accurate; lower them if it
overshoots.

## Using Claude on another computer

Open this repo folder in Claude Code (desktop app: choose the folder; terminal:
`cd ~/Robotics/93601V-repo` then `claude`). Claude reads `CLAUDE.md`, which
points it here, so it starts with the full picture. Ask it to
"update HANDOFF.md and sync" when you're done.

## Where we're up to

Newest entry first. Add a line whenever something changes or gets tested.

- **26 Nov 2026:** Simplified `TankComp.cpp` (about 130 lines shorter, easier
  to read). The robot does exactly the same thing: all 93 simulator runs
  (every routine, start and fault case) match the old version exactly.
- **26 Nov 2026:** Auton accuracy rework in `TankComp` (not yet run on the
  robot): autons now drive with closed-loop speed control instead of voltage;
  drives and turns use a smooth stopping curve and stop within 0.5" / 1 degree
  (were 1" / 2 degrees); the inertial is checked at start-up, can be
  recalibrated with CAL, and if it is missing or unplugged the robot steers by
  its wheels; a safety stop ends the auton if the robot turns the wrong way.
  New routines CHECK ROBOT, TUNE: DRIVE 48in, TUNE: TURN 90 x4 (see "Making the
  auton accurate"). The old `FORWARD_SCALE` comment had the direction backwards;
  fixed. Simulated: 48" drive within 0.3", turns within 0.3 degrees.
- **26 Nov 2026:** Fixed "memory permission error 0380619C" on the brain when
  downloading from VS Code. The brain object was created after the claw (which
  uses `Brain.ThreeWirePort.A`), because it was in a separate file. It is now
  created at the top of `src/main.cpp`, before `TankComp.cpp` is included.
  Checked in the built program: the brain is now set up first.
- **26 Nov 2026:** The repo is now a VEX VS Code project for TankComp
  (`makefile`, `vex/`, `include/vex.h`, `src/`, `.vscode/vex_project_settings.json`).
  Built with the VEX toolchain: same program size as the VEXcode build, no errors.
- **26 Nov 2026:** Both tank programs built with VEXcode's own compiler: no
  errors or warnings. VS Code's red underlines were it not finding `vex.h`;
  the VEX extension sets that up itself once the repo is a VEX project (it
  writes `.vscode/c_cpp_properties.json` on each computer; don't hand-edit it,
  a wrong format stops downloads with "Cannot read properties of undefined").
- **26 Nov 2026:** Side toggles slowed to 50% (`TOGGLE_SPEED`). Cleaned up
  `main`: old robot and X-drive code moved to the `deprecated` branch. Added
  `.github/copilot-instructions.md` so GitHub Copilot in VS Code knows the project.
- **25 Nov 2026:** Added this handoff setup (`HANDOFF.md`, `CLAUDE.md`,
  `scripts/`). VS Code and the VEX extension are set up on Lincoln's Mac.
- **24 Nov 2026:** Tank drive moved to new ports (drive 3, 11, 8, 20; inertial
  16; lift 1 and 10; side toggles 2 and 9) and to voltage control. Auton
  minimum powers raised to `MIN_MOVE` 20, `MIN_TURN` 15; retune on the robot.
- **19 Nov 2026:** Robot changed from X-drive to a 44W tank drive.

### The current robot

Tank drive. Use **`TankDrive.v5cpp`** for driver practice and
**`TankComp.v5cpp`** for matches and skills. Ports are in the header at the top
of each file and in `README.md`.

### Not done yet

- [ ] Run the steps in "Making the auton accurate" on the robot and put the
      numbers they give into `TankComp.cpp`: `DRIVE_SIGN`, `TRACK_WIDTH`,
      `FORWARD_SCALE`, `IMU_SCALE`.
- [ ] Measure and fill in: wheel diameter, claw reach, every lift height
      (`H_...`), `TOGGLE_FLIP_DEGREES`.
- [ ] Nothing else. Old code (X-drive and the earlier robot) is archived on the
      `deprecated` branch if it's ever needed again.
