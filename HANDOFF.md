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
- To download to the brain: open the `.v5cpp` in VEXcode V5.
- VS Code's red error underlines need the VEX V5 SDK, which the VEX extension
  downloads the first time you create or open a VEX project in VS Code. The
  repo's `.vscode` settings then point VS Code at it. If `vex.h` shows as "not
  found", do that once and reload VS Code (Cmd/Ctrl+Shift+P → Reload Window).
- In VEXcode, keep the **Devices panel empty** and never add `brain Brain;`.
  Every device is declared in the code; doing either gives "redefinition" errors.

## Using Claude on another computer

Open this repo folder in Claude Code (desktop app: choose the folder; terminal:
`cd ~/Robotics/93601V-repo` then `claude`). Claude reads `CLAUDE.md`, which
points it here, so it starts with the full picture. Ask it to
"update HANDOFF.md and sync" when you're done.

## Where we're up to

Newest entry first. Add a line whenever something changes or gets tested.

- **26 Nov 2026:** Both tank programs built with VEXcode's own compiler: no
  errors or warnings. VS Code's red underlines were it not finding `vex.h`;
  fixed with `.vscode/c_cpp_properties.json`.
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

- [ ] None of the auton code has run on the robot. First: check `DRIVE_SIGN` and
      `TURN_SIGN` (stick forward must drive forward, stick right must turn
      clockwise), then run the TEST routine (X + Y on the controller). The
      tuning steps are at the top of `TankComp.cpp`.
- [ ] Measure and fill in: wheel diameter, claw reach, every lift height
      (`H_...`), `TOGGLE_FLIP_DEGREES`.
- [ ] Nothing else. Old code (X-drive and the earlier robot) is archived on the
      `deprecated` branch if it's ever needed again.
