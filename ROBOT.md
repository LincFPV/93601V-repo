# 93601V Robot: Handoff Notes

> **Update 19 Nov 2026: the robot is now a 44W tank drive, not an X-drive.** Left side motors 10 (front) and 9 (back), right side 20 (front) and 19 (back), geared 36:60. Driver code is `TankDrive.v5cpp` (arcade: left stick forward/back, right stick turn; no strafe, no field-centric, A and B unused). Competition code is `OverrideComp.v5cpp`. `XDrive.*` has been removed. Most of the notes below describe the older X-drive version.


Everything known about the robot and its code as of 10 November 2026, so work can continue on another computer (or in a new Claude session: paste or attach this file).

## Files in this package

| File | What it is |
|---|---|
| `ROBOT.md` | This file. The full code is also copied at the bottom. |
| `XDrive.v5cpp` | The code as a VEXcode V5 project. Open this in VEXcode V5. |
| `XDrive.cpp` | The same code as plain C++ text. |

## Team and repo

- Team: **93601V** (VEX V5).
- GitHub repo: https://github.com/LincFPV/93601V-repo (branch `main`).
- The repo already holds older code for a previous robot: `drivetrain.v5cpp`, `auton.v5cpp`, `Comp.v5cpp`, `CompV2.v5cpp`, `README.md`. That robot was a 4-motor tank drive with a lift, claw and spin claw. The X-drive code is separate from those files.
- **The X-drive code has not been pushed by Claude.** The push failed because the Claude desktop app blocks the GitHub sign-in prompt. The plan was to upload `XDrive.v5cpp` and `XDrive.cpp` by hand on the GitHub website (Add file -> Upload files). Check the repo to see whether the latest version is there.
- Suggested README line for the repo:
  `- XDrive.v5cpp --- X-drive competition code with field-centric driving (FL 10, FR 20, BL 9, BR 19, inertial 12, radio 11), 33W cascade lift (1, 2, 3), pneumatic claw (3-wire A) and a toggle (4) that runs until the colour sensor (5) sees red. XDrive.cpp is the same code as plain C++`

## The robot

- **Drive:** X-drive, 4 x 11W V5 motors (44W total), one per wheel.
- **Lift:** cascade lift, 2 x 11W motors (22W) on ports 1 and 2. (Changed 17 Nov 2026 from 3 motors; port 3 is now free. The drive is geared 36:60. The code copy at the bottom of this file is the older 10 Nov version; use XDrive.cpp.)
- **Toggles (changed 17 Nov 2026):** the single toggle motor and the colour sensor are gone. There are now two side toggles, one 5.5W motor each: left on port 3, right on port 4. Hold R2 to spin both; hold the Up arrow to spin both the other way. Port 5 is free. The "Active toggle", port table, controls and "toggle does not work" sections below describe the old mechanism.
- **Claw:** pneumatic, one solenoid.
- **Active toggle:** one motor, watched by a colour (optical) sensor.
- **Inertial sensor** for field-centric driving.

### Ports

| Device | Port | Notes |
|---|---|---|
| Front left drive | 10 | not reversed |
| Back left drive | 9 | not reversed |
| Front right drive | 20 | reversed |
| Back right drive | 19 | reversed |
| Radio (antenna) | 11 | no code needed |
| Inertial | 12 | given by the team |
| Lift motor A | 1 | not reversed |
| Lift motor B | 2 | reversed (a guess) |
| Lift motor C | 3 | not reversed |
| Toggle motor | 4 | team confirmed it is plugged into 4 |
| Colour sensor | 5 | chosen by Claude, not confirmed on the robot |
| Claw solenoid | 3-wire A | chosen by Claude, not confirmed on the robot |

Drive ports and the inertial port came from the team. Lift ports 1-3, sensor port 5 and 3-wire A were picked by Claude when asked to "pick some that make sense".

All motors are set to `ratio18_1` (200 RPM green cartridge). If the robot uses blue 600 RPM cartridges, change to `ratio6_1`.

### Controls

| Control | Action |
|---|---|
| Left stick up/down | drive forward / back (inverted in code, see below) |
| Left stick left/right | strafe |
| Right stick left/right | turn |
| L1 / L2 | lift up / down, holds position when released |
| R1 | open / close claw (starts open) |
| R2 | run toggle until the colour sensor sees red; press again to cancel |
| A | switch field-centric / robot-centric (starts field-centric) |
| B | reset heading (face the robot away from the driver first) |

## How the code works

- VEXcode V5 C++ with the competition template (`pre_auton`, `autonomous`, `usercontrol`).
- All devices are declared in the code. **Keep the VEXcode Devices panel empty**, or you get "redefinition" errors.
- **Do not add `brain Brain;`** to the code. VEXcode adds its own, and a second one gives `redefinition of 'Brain'`.
- X-drive mixing: `fl = drive + strafe + turn`, `fr = drive - strafe - turn`, `bl = drive - strafe + turn`, `br = drive + strafe - turn`, scaled down if any wheel goes over 100%.
- **Direction state right now:** inside `xDrive`, `drive` is negated (forward/back inverted). Strafe is **not** negated and turn is not negated. History: the team asked to invert both forward/back and left/right, then later asked to invert strafing again, which put strafe back to the original sign.
- Because the inversion is inside `xDrive`, autonomous is affected the same way as driver control.
- Field-centric: the joystick input is rotated by the inertial heading before mixing. Not yet checked on the robot after the inversion changes. Test: press B facing away, turn the robot 90 degrees, push the stick up; it should still drive away from the driver.
- Motors spin with `percent` velocity. For more speed and punch, switch to voltage: `spin(fwd, value * 0.12, volt)`.
- Toggle macro: R2 starts `ToggleMotor` at `TOGGLE_SPEED` (60%). Each loop, `updateToggleMacro()` stops it when `seesRed()` is true (one short buzz) or after `TOGGLE_TIMEOUT_MS` (3000 ms, long buzz). `seesRed()` needs `isNearObject()` and hue below 20 or above 340.
- R2 is read by edge detection in the drive loop. A, B and R1 callbacks are registered once in `main` so a restart of driver control cannot double-register them.
- The brain screen shows a live toggle readout during driver control: R2 presses, toggle running/stopped, motor port 4 found/missing, sensor port 5 found/missing, hue and near.
- Autonomous is only a placeholder: drive 1 second, strafe 1 second, stop.

## Open problems

1. **The toggle does not work.** Pressing R2 gave no spin and no buzz, with the motor on port 4. Cause not found yet. Next step: build the current code, press R2 once and read the brain screen:
   - `R2 presses` not going up -> the code is not seeing the button (old program downloaded, or controller not linked).
   - `Toggle` stays `stopped` -> the macro cancels itself; a code bug. One known flaw: if the sensor already sees red when R2 is pressed, it stops at once.
   - `Toggle` says `RUNNING` but nothing moves -> motor, cable or mechanism.
   - `Motor port 4: MISSING` or `Sensor port 5: MISSING` -> wiring or wrong port.
   - Note the `Hue` number with the red side in front of the sensor, to tune `seesRed()`.
   - It was never confirmed that the toggle is motor-driven rather than pneumatic.
2. **The current code has not been confirmed to build.** Two build errors were found and fixed so far: `redefinition of 'Brain'`, and `printAt is ambiguous` (fixed by adding `true` as the third argument). The latest version has not been built since the last edits.
3. **Nothing has been confirmed on the robot** except that the earlier version built and drove well enough for the team to ask for direction changes. Still to check: lift motor directions, claw open/close direction, toggle direction, field-centric after the inversions.
4. **GitHub** may not have the latest files.

## Ideas discussed for more speed

- Blue 600 RPM cartridges (then change `ratio18_1` to `ratio6_1`), bigger wheels, less friction and weight.
- Voltage control instead of percent velocity.
- Coast instead of brake when stopping.
- X-drives are about 1.4x faster on the diagonal.

## Full code (same as XDrive.cpp)

```cpp
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Team 93601V - X-Drive (4 x 11W motors)                                  */
/*                                                                            */
/*    Ports:                                                                  */
/*      Radio (antenna) ... 11  (no code needed, just plug it in)             */
/*      Inertial .......... 12                                                */
/*      Front Left ........ 10                                                */
/*      Front Right ....... 20                                                */
/*      Back Left .........  9                                                */
/*      Back Right ........ 19                                                */
/*      Lift (3 x 11W) .... 1, 2, 3   <- change these to match your robot     */
/*      Toggle motor ......  4        <- change to match your robot           */
/*      Colour sensor .....  5        <- change to match your robot           */
/*      Claw piston ....... 3-wire port A on the brain                        */
/*                                                                            */
/*    Controls:                                                               */
/*      Left stick Y  = forward / backward                                    */
/*      Left stick X  = strafe left / right                                   */
/*      Right stick X = turn                                                  */
/*      L1 / L2       = lift up / down                                        */
/*      R1            = open / close claw                                     */
/*      R2            = run toggle until the colour sensor sees red           */
/*                      (press R2 again to cancel)                            */
/*      A             = toggle field-centric / robot-centric                  */
/*      B             = reset heading (point robot "forward" first)           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

using namespace vex;

competition Competition;
controller Controller1 = controller(primary);

// Right side is reversed so positive power drives every wheel "forward".
motor FrontLeft  = motor(PORT10, ratio18_1, false);
motor FrontRight = motor(PORT20, ratio18_1, true);
motor BackLeft   = motor(PORT9,  ratio18_1, false);
motor BackRight  = motor(PORT19, ratio18_1, true);

inertial Inertial = inertial(PORT12);

// 33W cascade lift: three 11W motors working together.
// If one motor fights the others, flip its true/false.
motor LiftMotorA = motor(PORT1, ratio18_1, false);
motor LiftMotorB = motor(PORT2, ratio18_1, true);
motor LiftMotorC = motor(PORT3, ratio18_1, false);
motor_group Lift = motor_group(LiftMotorA, LiftMotorB, LiftMotorC);

// Pneumatic claw: solenoid plugged into 3-wire port A.
digital_out Claw = digital_out(Brain.ThreeWirePort.A);

// Active toggle and the colour (optical) sensor that watches it.
motor ToggleMotor = motor(PORT4, ratio18_1, false);
optical ColourSensor = optical(PORT5);

// Joystick values below this are treated as 0 to stop drift.
const int DEADBAND = 5;

// Toggle macro settings.
const int TOGGLE_SPEED = 60;          // percent
const int TOGGLE_TIMEOUT_MS = 3000;   // give up if red is never seen

bool fieldCentric = true;
bool clawClosed = false;
bool toggleRunning = false;
int toggleStartTime = 0;

int applyDeadband(int value) {
  return (std::abs(value) < DEADBAND) ? 0 : value;
}

// drive, strafe, turn are -100..100 (percent)
void xDrive(double drive, double strafe, double turn) {
  // Forward/back is inverted; strafing and turning are not.
  drive = -drive;

  double fl = drive + strafe + turn;
  double fr = drive - strafe - turn;
  double bl = drive - strafe + turn;
  double br = drive + strafe - turn;

  // Scale everything down if any wheel is asked for more than 100%
  // so the robot keeps the right direction at full stick.
  double maxPower = std::fmax(std::fmax(std::fabs(fl), std::fabs(fr)),
                              std::fmax(std::fabs(bl), std::fabs(br)));
  if (maxPower > 100) {
    fl = fl / maxPower * 100;
    fr = fr / maxPower * 100;
    bl = bl / maxPower * 100;
    br = br / maxPower * 100;
  }

  FrontLeft.spin(fwd, fl, percent);
  FrontRight.spin(fwd, fr, percent);
  BackLeft.spin(fwd, bl, percent);
  BackRight.spin(fwd, br, percent);
}

void stopDrive() {
  FrontLeft.stop(brake);
  FrontRight.stop(brake);
  BackLeft.stop(brake);
  BackRight.stop(brake);
}

/*---------------------------------------------------------------------------*/
/*                          Claw and Toggle                                  */
/*---------------------------------------------------------------------------*/
void setClaw(bool closed) {
  clawClosed = closed;
  Claw.set(closed);
}

void toggleClaw() {
  setClaw(!clawClosed);
}

// Red sits at both ends of the hue wheel (0-360), so check both ends.
bool seesRed() {
  if (!ColourSensor.isNearObject()) {
    return false;
  }
  double hue = ColourSensor.hue();
  return hue < 20 || hue > 340;
}

void stopToggle() {
  toggleRunning = false;
  ToggleMotor.stop(hold);
}

// R2: start the toggle macro, or cancel it if it's already running.
void startToggleMacro() {
  if (toggleRunning) {
    stopToggle();
    return;
  }
  toggleRunning = true;
  toggleStartTime = Brain.Timer.time(msec);
  ToggleMotor.spin(fwd, TOGGLE_SPEED, percent);
}

// Called every loop so the toggle runs without freezing the drive.
void updateToggleMacro() {
  if (!toggleRunning) {
    return;
  }
  if (seesRed()) {
    stopToggle();
    Controller1.rumble(".");
  } else if (Brain.Timer.time(msec) - toggleStartTime > TOGGLE_TIMEOUT_MS) {
    stopToggle();
    Controller1.rumble("--");
  }
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous                                   */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  Lift.setStopping(hold);
  ToggleMotor.setStopping(hold);
  setClaw(false);

  // Turn on the sensor's light so it reads colour the same in any lighting.
  ColourSensor.setLight(ledState::on);
  ColourSensor.setLightPower(100, percent);

  Brain.Screen.clearScreen();
  Brain.Screen.printAt(10, 30, "Calibrating inertial... don't move!");
  Inertial.calibrate();
  while (Inertial.isCalibrating()) {
    wait(50, msec);
  }
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(10, 30, "93601V ready");
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous                                   */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  // Example: drive forward 1 second, strafe right 1 second, stop.
  xDrive(50, 0, 0);
  wait(1000, msec);
  xDrive(0, 50, 0);
  wait(1000, msec);
  stopDrive();
}

/*---------------------------------------------------------------------------*/
/*                              User Control                                 */
/*---------------------------------------------------------------------------*/
void toggleFieldCentric() {
  fieldCentric = !fieldCentric;
  Controller1.Screen.clearLine(1);
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(fieldCentric ? "Field-centric" : "Robot-centric");
}

void resetHeading() {
  Inertial.setHeading(0, degrees);
  Controller1.rumble(".");
}

// Shows on the brain what the toggle macro can see, for troubleshooting.
void showToggleStatus(int r2Presses) {
  // The "true" draws each line over the old one and picks the right printAt.
  Brain.Screen.printAt(10, 60, true, "R2 presses: %d   ", r2Presses);
  Brain.Screen.printAt(10, 85, true, "Toggle: %s   ", toggleRunning ? "RUNNING" : "stopped");
  Brain.Screen.printAt(10, 110, true, "Motor port 4: %s   ", ToggleMotor.installed() ? "found" : "MISSING");
  Brain.Screen.printAt(10, 135, true, "Sensor port 5: %s   ", ColourSensor.installed() ? "found" : "MISSING");
  Brain.Screen.printAt(10, 160, true, "Hue: %.0f  Near: %s   ", ColourSensor.hue(), ColourSensor.isNearObject() ? "yes" : "no");
}

void usercontrol(void) {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(fieldCentric ? "Field-centric" : "Robot-centric");

  bool r2WasPressed = false;
  int r2Presses = 0;

  while (true) {
    // R2 is checked here so one press can only ever start or cancel once.
    bool r2Pressed = Controller1.ButtonR2.pressing();
    if (r2Pressed && !r2WasPressed) {
      r2Presses++;
      startToggleMacro();
    }
    r2WasPressed = r2Pressed;

    double drive  = applyDeadband(Controller1.Axis3.position());
    double strafe = applyDeadband(Controller1.Axis4.position());
    double turn   = applyDeadband(Controller1.Axis1.position());

    if (fieldCentric) {
      // Rotate the joystick input by the robot's heading so "up" on the
      // stick always drives away from the driver.
      double headingRad = Inertial.heading(degrees) * M_PI / 180.0;
      double rotDrive  =  drive * std::cos(headingRad) + strafe * std::sin(headingRad);
      double rotStrafe = -drive * std::sin(headingRad) + strafe * std::cos(headingRad);
      drive  = rotDrive;
      strafe = rotStrafe;
    }

    if (drive == 0 && strafe == 0 && turn == 0) {
      stopDrive();
    } else {
      xDrive(drive, strafe, turn);
    }

    // Lift: hold L1 to go up, L2 to go down, holds position when released.
    if (Controller1.ButtonL1.pressing()) {
      Lift.spin(fwd, 100, percent);
    } else if (Controller1.ButtonL2.pressing()) {
      Lift.spin(reverse, 100, percent);
    } else {
      Lift.stop(hold);
    }

    updateToggleMacro();
    showToggleStatus(r2Presses);

    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Registered here, not in usercontrol, so each press only fires once
  // even when the field or switch restarts driver control.
  Controller1.ButtonA.pressed(toggleFieldCentric);
  Controller1.ButtonB.pressed(resetHeading);
  Controller1.ButtonR1.pressed(toggleClaw);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
```
