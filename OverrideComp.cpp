/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Team 93601V - X-Drive competition code for VEX V5 Override (2026-27)    */
/*                                                                            */
/*    Ports (same as XDrive.cpp):                                             */
/*      Front Left 10, Front Right 20, Back Left 9, Back Right 19             */
/*      Inertial 12, Radio 11        Drive geared 36:60 (motor:wheel)         */
/*      Lift 1, 2         Side toggles: left 3, right 4 (5.5W motors)         */
/*      Claw piston: 3-wire port A                                            */
/*                                                                            */
/*    Driver controls (unchanged):                                            */
/*      Left stick = drive / strafe      Right stick X = turn                 */
/*      L1 / L2 = lift up / down         R1 = claw                            */
/*      R2 (hold) = spin both side toggles                                    */
/*      Up arrow (hold) = spin both side toggles the other way                */
/*      A  = field-centric on/off        B = reset heading                    */
/*      X + Y together = test-run the selected auton (only works when NOT     */
/*                       plugged into field control). Down arrow = abort.     */
/*                                                                            */
/*    Before a match, on the brain screen pick:                               */
/*      1. Alliance (RED / BLUE)                                              */
/*      2. Start (BY STATION = the wall next to your drivers,                 */
/*                SIDE WALL  = the other wall on your side)                   */
/*      3. Routine (arrows)                                                   */
/*    For Autonomous Coding Skills pick the routine "SKILLS 60s".             */
/*                                                                            */
/*    HOW TO PLACE THE ROBOT (every routine except TEST):                     */
/*      Centred on the wall, right under the Toggle, with the side toggle     */
/*      picked in AUTON_TOGGLE_ON_RIGHT against the wall and the lift down.   */
/*      Preload in the claw, yellow end UP. Power on with the lift down       */
/*      and don't move the robot while the inertial calibrates.               */
/*                                                                            */
/*    TUNING ORDER (nothing below has been run on the real robot yet):        */
/*      1. Run routine "TEST" with X+Y. It should go forward 24", right       */
/*         24", turn 90 degrees clockwise, then come home. If a direction     */
/*         is wrong fix DRIVE_SIGN / STRAFE_SIGN / TURN_SIGN. If the          */
/*         distance is wrong fix WHEEL_DIAMETER (or the two SCALE numbers).   */
/*      2. Drive the lift by hand and read "Lift" on the brain screen to      */
/*         fill in every H_... height.                                        */
/*      3. Measure CLAW_REACH and START_WALL_DIST, set AUTON_TOGGLE_ON_RIGHT  */
/*         and tune TOGGLE_FLIP_DEGREES until one flip lands on your colour.  */
/*      4. Tune one routine at a time, shortest first.                        */
/*                                                                            */
/*    Keep the VEXcode Devices panel EMPTY and do not add "brain Brain;".     */
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

// 22W cascade lift (two 11W motors). If they fight each other, flip one true/false.
motor LiftMotorA = motor(PORT1, ratio18_1, false);
motor LiftMotorB = motor(PORT2, ratio18_1, true);
motor_group Lift = motor_group(LiftMotorA, LiftMotorB);

digital_out Claw = digital_out(Brain.ThreeWirePort.A);

// Side toggle mechanisms: one 5.5W motor on each side of the robot.
// (5.5W motors always run at 200 RPM, so ratio18_1 is correct for them.)
// If one spins the wrong way, flip its true/false.
motor ToggleLeft  = motor(PORT3, ratio18_1, false);
motor ToggleRight = motor(PORT4, ratio18_1, true);

/*---------------------------------------------------------------------------*/
/*                         NUMBERS TO TUNE                                   */
/*---------------------------------------------------------------------------*/

// ---- Driver ----
const int DEADBAND = 5;

// ---- Side toggles ----
const int    TOGGLE_SPEED = 100;            // percent (5.5W motors are weak)
const bool   AUTON_TOGGLE_ON_RIGHT = true;  // which side toggle the autons use
const double TOGGLE_FLIP_DEGREES = 360;     // motor degrees for one auton flip. TUNE.
const int    TOGGLE_AUTON_TIMEOUT_MS = 2000;

// ---- Drive base ----
const double WHEEL_DIAMETER = 3.25;     // inches. MEASURE YOUR WHEELS.
// Drive gearing: a 36-tooth gear on each motor turns a 60-tooth gear on the
// wheel, so the wheel turns 0.6 times for every motor turn.
// If it is the other way round (60 on the motor), swap to 60.0 / 36.0.
const double DRIVE_GEAR_RATIO = 36.0 / 60.0;
const double FORWARD_SCALE  = 1.0;      // raise if the robot drives short
const double STRAFE_SCALE   = 1.0;      // raise if the robot strafes short
// These match the directions the team settled on in driver control.
const double DRIVE_SIGN  = -1;          // forward/back is inverted
const double STRAFE_SIGN =  1;
const double TURN_SIGN   = -1;          // turning is inverted

// ---- Robot shape (inches / degrees) ----
// Angles are measured clockwise from the FRONT of the robot, looking down.
const double CLAW_ANGLE        = 0;     // the claw is on the front
const double TOGGLE_MECH_ANGLE = AUTON_TOGGLE_ON_RIGHT ? 90 : 270;  // side put against the wall
const double CLAW_REACH        = 12;    // robot centre -> middle of the claw
const double START_WALL_DIST   = 9;     // robot centre -> wall at the start
const double TOGGLE_WALL_DIST  = 9;     // robot centre -> wall while flipping
const double WALL_CLEARANCE    = 8;     // how far to pull off the wall after
const double BACKOFF           = 5;     // line up this far back, then creep in

// ---- Lift heights (motor degrees, 0 = fully down) ----
const double H_CARRY        = 150;      // travelling with something held
const double H_GRAB_FLOOR   = 0;        // grab a cup that sits on the tiles
const double H_GRAB_WALLPIN = 220;      // grab the pin standing on a wall cup
const double H_LIFT_OFF     = 200;      // extra lift after grabbing / before placing
const double H_ALLIANCE_1   = 250;      // pin straight into an alliance goal
const double H_ALLIANCE_2   = 700;      // cup + pin on top of one pin
const double H_ALLIANCE_3   = 1150;     // cup + pin on top of two pins
const double H_NEUTRAL_1    = 250;      // pin straight into a short neutral goal
const double H_NEUTRAL_2    = 700;      // cup + pin on top of one pin
const int    LIFT_SPEED     = 100;      // percent
const int    GRAB_MS        = 200;      // time for the claw to close
const int    RELEASE_MS     = 200;      // time for the claw to open

// ---- Autonomous driving ----
const double FAST = 90;                 // percent
const double SLOW = 40;
const double MOVE_KP = 6.0;             // percent per inch of error
const double MOVE_KD = 0.15;
const double TURN_KP = 2.0;             // percent per degree of error
const double TURN_KD = 0.10;
const double MIN_MOVE = 10;             // smallest power that still moves
const double MIN_TURN = 6;
const double PASS_SPEED = 30;           // speed carried through a waypoint
const double SLEW = 4;                  // max power change per 10 ms
const double POS_TOL = 1.0;             // inches
const double HEAD_TOL = 2.0;            // degrees
const int    SETTLE_MS = 80;

// Skills: also put a third level on the first red goal.
const bool SKILLS_THIRD_LEVEL = true;

/*---------------------------------------------------------------------------*/
/*   The field. Inches from the centre, +X toward the blue station, +Y "up"  */
/*   in the game manual's overhead picture. Directions are compass-style:    */
/*   0 = +Y, 90 = +X, clockwise. Everything is written for RED starting BY   */
/*   THE STATION (left wall); the code mirrors it for the other three starts.*/
/*---------------------------------------------------------------------------*/
const double TOGGLE_X = -72, TOGGLE_Y = 0;
const double GOAL_A_X = -48, GOAL_A_Y = -24;   // alliance-coloured goal
const double GOAL_N_X = -48, GOAL_N_Y =  24;   // neutral goal (starts with a yellow pin)
const double STACK_IN_X  = -24, STACK_IN_Y  = -24;  // clear cup + yellow pin
const double STACK_OUT_X = -48, STACK_OUT_Y = -48;  // clear cup + yellow pin
const double WALL_A_X = -69, WALL_A_Y = -24;   // wall cup + yellow pin, alliance-goal side
const double WALL_N_X = -69, WALL_N_Y =  24;   // wall cup + yellow pin, neutral-goal side

// Where the robot stands (direction from the target back to the robot).
const double DIR_GOAL_A_OPEN   = 45;    // alliance goal, from the open-field side
const double DIR_GOAL_A_WALL   = 320;   // alliance goal, from the wall corner
const double DIR_GOAL_A_OUTER  = 135;   // alliance goal, from the outer stack side
const double DIR_GOAL_N_WALL   = 220;   // neutral goal, from the wall corner
const double DIR_STACK_IN      = 315;
const double DIR_STACK_OUT     = 45;
const double DIR_WALL_A        = 50;
const double DIR_WALL_N        = 130;
// Open spots where the robot has room to spin before working in the tight
// corner between a wall stack and the goal next to it.
const double STAGE_X = -56, STAGE_A_Y = -6, STAGE_N_Y = 6;

/*---------------------------------------------------------------------------*/
/*                              Selections                                   */
/*---------------------------------------------------------------------------*/
const int R_NONE           = 0;
const int R_TOGGLE         = 1;
const int R_PRELOAD        = 2;
const int R_TOGGLE_PRELOAD = 3;
const int R_FIELD_STACK    = 4;
const int R_WALL_STACK     = 5;
const int R_FULL           = 6;
const int R_SKILLS         = 7;
const int R_TEST           = 8;
const int ROUTINE_COUNT    = 9;

const char *routineName(int r) {
  switch (r) {
    case R_NONE:           return "DO NOTHING";
    case R_TOGGLE:         return "TOGGLE ONLY";
    case R_PRELOAD:        return "PRELOAD ONLY";
    case R_TOGGLE_PRELOAD: return "TOGGLE + PRELOAD";
    case R_FIELD_STACK:    return "TOG+PRE+FIELD STACK";
    case R_WALL_STACK:     return "TOG+PRE+WALL STACK";
    case R_FULL:           return "FULL QUADRANT";
    case R_SKILLS:         return "SKILLS 60s";
    case R_TEST:           return "TEST (24in square)";
  }
  return "?";
}

bool blueAlliance = false;      // picked on the brain screen
bool sideQuadrant = false;      // false = start by your alliance station
int  routine = R_TOGGLE;
bool selectorDirty = true;

bool fieldCentric = true;
bool clawClosed = false;
double driverZero = 0;          // heading that counts as "away from the driver"

int nowMs() {
  return (int)Brain.Timer.time(msec);
}

int applyDeadband(int value) {
  return (std::abs(value) < DEADBAND) ? 0 : value;
}

double wrap180(double a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

/*---------------------------------------------------------------------------*/
/*                                 Drive                                     */
/*---------------------------------------------------------------------------*/
// drive = forward, strafe = right, turn = clockwise, all -100..100 percent
void xDrive(double drive, double strafe, double turn) {
  drive  = DRIVE_SIGN  * drive;
  strafe = STRAFE_SIGN * strafe;
  turn   = TURN_SIGN   * turn;

  double fl = drive + strafe + turn;
  double fr = drive - strafe - turn;
  double bl = drive - strafe + turn;
  double br = drive + strafe - turn;

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
/*      Odometry: where the robot is, from the inertial + wheel encoders     */
/*---------------------------------------------------------------------------*/
double robotX = 0, robotY = 0;      // inches, field coordinates
double headingOffset = 0;
double prevFL = 0, prevFR = 0, prevBL = 0, prevBR = 0, prevHeading = 0;

// Field heading in degrees, clockwise, not wrapped.
double robotHeading() {
  return Inertial.rotation(degrees) + headingOffset;
}

void setPose(double x, double y, double heading) {
  robotX = x;
  robotY = y;
  headingOffset = heading - Inertial.rotation(degrees);
  prevHeading = heading;
  prevFL = FrontLeft.position(degrees);
  prevFR = FrontRight.position(degrees);
  prevBL = BackLeft.position(degrees);
  prevBR = BackRight.position(degrees);
}

void updateOdometry() {
  double fl = FrontLeft.position(degrees);
  double fr = FrontRight.position(degrees);
  double bl = BackLeft.position(degrees);
  double br = BackRight.position(degrees);
  double dFL = fl - prevFL, dFR = fr - prevFR, dBL = bl - prevBL, dBR = br - prevBR;
  prevFL = fl; prevFR = fr; prevBL = bl; prevBR = br;

  // X-drive wheels sit at 45 degrees, so the robot travels sqrt(2) times
  // as far as each wheel rolls.
  double inchesPerDegree = M_PI * WHEEL_DIAMETER / 360.0 * DRIVE_GEAR_RATIO * 1.41421356;
  double ahead  = DRIVE_SIGN  * (dFL + dFR + dBL + dBR) / 4.0 * inchesPerDegree * FORWARD_SCALE;
  double across = STRAFE_SIGN * (dFL - dFR - dBL + dBR) / 4.0 * inchesPerDegree * STRAFE_SCALE;

  double heading = robotHeading();
  double mid = (heading + prevHeading) / 2.0 * M_PI / 180.0;
  prevHeading = heading;

  robotX += ahead * std::sin(mid) + across * std::cos(mid);
  robotY += ahead * std::cos(mid) - across * std::sin(mid);
}

/*---------------------------------------------------------------------------*/
/*                       Claw and side toggles                               */
/*---------------------------------------------------------------------------*/
void setClaw(bool closed) {
  clawClosed = closed;
  Claw.set(closed);
}

void toggleClaw() {
  setClaw(!clawClosed);
}

void stopToggles() {
  ToggleLeft.stop(hold);
  ToggleRight.stop(hold);
}

/*---------------------------------------------------------------------------*/
/*                     Autonomous building blocks                            */
/*---------------------------------------------------------------------------*/
int  autonDeadline = 0;     // every step gives up at this time
bool testRun = false;       // started from the controller, not the field
bool tfBlue = false;        // how the routine is mirrored onto the field
bool tfMirror = false;

int timeLeft() {
  return autonDeadline - nowMs();
}

// Waits while keeping the position estimate up to date.
void waitMs(int ms) {
  int end = nowMs() + ms;
  do {
    updateOdometry();
    if (testRun && Controller1.ButtonDown.pressing()) {
      autonDeadline = 0;
    }
    wait(10, msec);
  } while (nowMs() < end && timeLeft() > 0);
}

// Routine coordinates -> real field coordinates.
double fieldX(double cx, double cy) {
  double x = tfMirror ? cy : cx;
  return tfBlue ? -x : x;
}
double fieldY(double cx, double cy) {
  double y = tfMirror ? cx : cy;
  return tfBlue ? -y : y;
}
double fieldDir(double dir) {
  if (tfMirror) dir = 90 - dir;
  if (tfBlue) dir += 180;
  return dir;
}

// Drive to a field position and heading, all three at once.
// passThrough = slow down but don't stop and settle, just get close and carry on.
void moveField(double tx, double ty, double th, double maxSpeed, int timeoutMs, bool passThrough) {
  if (timeLeft() <= 0) return;
  int end = nowMs() + timeoutMs;
  double lastSpeed = 0;
  double prevDist = std::sqrt((tx - robotX) * (tx - robotX) + (ty - robotY) * (ty - robotY));
  double prevErr = wrap180(th - robotHeading());
  int settled = 0;
  double posTol = passThrough ? 3.0 : POS_TOL;
  double headTol = passThrough ? 10.0 : HEAD_TOL;

  while (nowMs() < end && timeLeft() > 0) {
    updateOdometry();
    if (testRun && Controller1.ButtonDown.pressing()) {
      autonDeadline = 0;
    }

    double ex = tx - robotX;
    double ey = ty - robotY;
    double dist = std::sqrt(ex * ex + ey * ey);
    double headErr = wrap180(th - robotHeading());

    if (dist < posTol && std::fabs(headErr) < headTol) {
      if (passThrough) break;
      settled += 10;
      if (settled >= SETTLE_MS) break;
    } else {
      settled = 0;
    }

    double speed = MOVE_KP * dist + MOVE_KD * (dist - prevDist) / 0.010;
    if (passThrough && speed < PASS_SPEED) speed = PASS_SPEED;   // keep rolling
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed > lastSpeed + SLEW) speed = lastSpeed + SLEW;
    if (dist > posTol && speed < MIN_MOVE) speed = MIN_MOVE;
    if (dist <= posTol * 0.5 || speed < 0) speed = 0;
    lastSpeed = speed;
    prevDist = dist;

    double turn = TURN_KP * headErr + TURN_KD * (headErr - prevErr) / 0.010;
    prevErr = headErr;
    if (turn > maxSpeed) turn = maxSpeed;
    if (turn < -maxSpeed) turn = -maxSpeed;
    if (std::fabs(headErr) > headTol && std::fabs(turn) < MIN_TURN) {
      turn = (headErr > 0) ? MIN_TURN : -MIN_TURN;
    }

    // Turn the field direction into the robot's own forward / right.
    double h = robotHeading() * M_PI / 180.0;
    double ahead = 0, across = 0;
    if (dist > 0.01) {
      ahead  = (ex * std::sin(h) + ey * std::cos(h)) / dist * speed;
      across = (ex * std::cos(h) - ey * std::sin(h)) / dist * speed;
    }
    xDrive(ahead, across, turn);
    wait(10, msec);
  }
  if (!passThrough) stopDrive();
}

// Go to a routine point with a chosen side of the robot (mechAngle) facing
// the routine direction faceDir.
void goTo(double cx, double cy, double faceDir, double mechAngle, double speed, int timeoutMs, bool passThrough) {
  moveField(fieldX(cx, cy), fieldY(cx, cy), fieldDir(faceDir) - mechAngle, speed, timeoutMs, passThrough);
}

// Stand "reach" inches from a target, on its fromDir side, with one side of
// the robot pointing at the target.
void standOff(double tx, double ty, double fromDir, double reach, double mechAngle, double speed, int timeoutMs, bool passThrough) {
  double r = fromDir * M_PI / 180.0;
  goTo(tx + reach * std::sin(r), ty + reach * std::cos(r), fromDir + 180, mechAngle, speed, timeoutMs, passThrough);
}

void liftTo(double height, bool waitForIt) {
  if (timeLeft() <= 0) return;
  Lift.setVelocity(LIFT_SPEED, percent);
  Lift.spinToPosition(height, degrees, false);
  if (!waitForIt) return;
  int end = nowMs() + 1500;
  while (std::fabs(LiftMotorA.position(degrees) - height) > 15 && nowMs() < end && timeLeft() > 0) {
    waitMs(10);
  }
}

void autonClaw(bool closed) {
  if (timeLeft() <= 0) return;
  setClaw(closed);
}

// Spin the side toggle that is against the wall by one flip. There is no
// colour sensor now, so this is a fixed amount: start every run with the
// field Toggle in the same position and tune TOGGLE_FLIP_DEGREES for it.
// reverseIt spins the other way (the way that lands on blue instead of red).
void flipToggle(bool reverseIt) {
  if (timeLeft() <= 0) return;
  motor &side = AUTON_TOGGLE_ON_RIGHT ? ToggleRight : ToggleLeft;
  double startPos = side.position(degrees);
  int start = nowMs();
  side.spin(reverseIt ? reverse : fwd, TOGGLE_SPEED, percent);
  while (std::fabs(side.position(degrees) - startPos) < TOGGLE_FLIP_DEGREES &&
         nowMs() - start < TOGGLE_AUTON_TIMEOUT_MS && timeLeft() > 0) {
    waitMs(10);
  }
  side.stop(hold);
}

// Flip the Toggle of the current quadrant, then pull off the wall (a Toggle
// only counts when no robot touches it).
void doToggle(bool wantBlue, bool alreadyAtWall) {
  if (!alreadyAtWall) {
    standOff(TOGGLE_X, TOGGLE_Y, 90, TOGGLE_WALL_DIST + WALL_CLEARANCE, TOGGLE_MECH_ANGLE, FAST, 3000, true);
  }
  standOff(TOGGLE_X, TOGGLE_Y, 90, TOGGLE_WALL_DIST, TOGGLE_MECH_ANGLE, SLOW, 1500, false);
  flipToggle(wantBlue);
  standOff(TOGGLE_X, TOGGLE_Y, 90, TOGGLE_WALL_DIST + WALL_CLEARANCE, TOGGLE_MECH_ANGLE, FAST, 1200, true);
}

// Pick something up with the claw.
void grabAt(double tx, double ty, double fromDir, double height) {
  if (timeLeft() <= 0) return;
  autonClaw(false);
  liftTo(height, false);
  standOff(tx, ty, fromDir, CLAW_REACH + BACKOFF, CLAW_ANGLE, FAST, 2500, true);
  liftTo(height, true);
  standOff(tx, ty, fromDir, CLAW_REACH, CLAW_ANGLE, SLOW, 1500, false);
  autonClaw(true);
  waitMs(GRAB_MS);
  liftTo(height + H_LIFT_OFF, true);
  standOff(tx, ty, fromDir, CLAW_REACH + BACKOFF, CLAW_ANGLE, SLOW, 1000, true);
}

// Put what the claw is holding onto a goal (or onto the stack on it).
void placeOn(double gx, double gy, double fromDir, double height) {
  if (timeLeft() <= 0) return;
  liftTo(height + H_LIFT_OFF, false);
  standOff(gx, gy, fromDir, CLAW_REACH + BACKOFF, CLAW_ANGLE, FAST, 2500, true);
  liftTo(height + H_LIFT_OFF, true);
  standOff(gx, gy, fromDir, CLAW_REACH, CLAW_ANGLE, SLOW, 1500, false);
  liftTo(height, true);
  autonClaw(false);
  waitMs(RELEASE_MS);
  standOff(gx, gy, fromDir, CLAW_REACH + BACKOFF, CLAW_ANGLE, SLOW, 1000, true);
  liftTo(H_CARRY, false);
}

void finishAuton() {
  stopDrive();
  stopToggles();
  Lift.stop(hold);
}

/*---------------------------------------------------------------------------*/
/*              Working in the corner by a wall stack                        */
/*---------------------------------------------------------------------------*/
// The wall stack and the goal beside it are only 21" apart, so the robot
// spins at an open "stage" spot first and then slides in without turning.
// allianceSide = the stack / goal pair on the alliance-goal side of the Toggle.

void cornerGrab(bool allianceSide, double height) {
  if (timeLeft() <= 0) return;
  double dir = allianceSide ? DIR_WALL_A : DIR_WALL_N;
  goTo(STAGE_X, allianceSide ? STAGE_A_Y : STAGE_N_Y, dir + 180, CLAW_ANGLE, FAST, 2500, false);
  grabAt(allianceSide ? WALL_A_X : WALL_N_X, allianceSide ? WALL_A_Y : WALL_N_Y, dir, height);
  // Back out to the open spot before anything turns.
  goTo(STAGE_X, allianceSide ? STAGE_A_Y : STAGE_N_Y, dir + 180, CLAW_ANGLE, FAST, 1500, true);
}

void cornerPlace(bool allianceSide, double height) {
  if (timeLeft() <= 0) return;
  double dir = allianceSide ? DIR_GOAL_A_WALL : DIR_GOAL_N_WALL;
  goTo(STAGE_X, allianceSide ? STAGE_A_Y : STAGE_N_Y, dir + 180, CLAW_ANGLE, FAST, 2500, false);
  placeOn(allianceSide ? GOAL_A_X : GOAL_N_X, allianceSide ? GOAL_A_Y : GOAL_N_Y, dir, height);
  goTo(STAGE_X, allianceSide ? STAGE_A_Y : STAGE_N_Y, dir + 180, CLAW_ANGLE, FAST, 1500, true);
}

/*---------------------------------------------------------------------------*/
/*                         15 second match routines                          */
/*---------------------------------------------------------------------------*/
// Each step is skipped if there isn't enough time left to finish it, so a
// slow run ends early and clear of the wall instead of half-way through.

void matchRoutine(int which) {
  bool useToggle = (which != R_PRELOAD);

  // 1. Toggle: makes every visible yellow pin half in this quadrant ours.
  //    The neutral goal already holds a yellow pin, so this alone is 10.
  if (useToggle) {
    doToggle(tfBlue, true);
  } else {
    standOff(TOGGLE_X, TOGGLE_Y, 90, TOGGLE_WALL_DIST + WALL_CLEARANCE, TOGGLE_MECH_ANGLE, FAST, 1200, true);
  }
  if (which == R_TOGGLE) return;

  // 2. Preload into the alliance goal.
  if (timeLeft() < 4000) return;
  if (which == R_WALL_STACK) {
    cornerPlace(true, H_ALLIANCE_1);
  } else {
    placeOn(GOAL_A_X, GOAL_A_Y, DIR_GOAL_A_OPEN, H_ALLIANCE_1);
  }
  if (which == R_PRELOAD || which == R_TOGGLE_PRELOAD) return;

  // 3. A cup + yellow pin on top of the preload: the goal now shows two
  //    scored halves.
  if (timeLeft() < 6000) return;
  if (which == R_WALL_STACK) {
    cornerGrab(true, H_GRAB_FLOOR);
    cornerPlace(true, H_ALLIANCE_2);
    return;
  }
  grabAt(STACK_IN_X, STACK_IN_Y, DIR_STACK_IN, H_GRAB_FLOOR);
  placeOn(GOAL_A_X, GOAL_A_Y, DIR_GOAL_A_OPEN, H_ALLIANCE_2);
  if (which == R_FIELD_STACK) return;

  // 4. FULL only: the wall cup + yellow pin onto the neutral goal's pin.
  if (timeLeft() < 6500) return;
  cornerGrab(false, H_GRAB_FLOOR);
  cornerPlace(false, H_NEUTRAL_2);
}

/*---------------------------------------------------------------------------*/
/*                       60 second skills routine                            */
/*---------------------------------------------------------------------------*/
// Starts in the quadrant by the red station (rule RSC2). Works the two red
// quadrants, then parks in the Midfield. Goals start empty in skills.
void skillsRoutine() {
  const int PARK_MS = 4000;       // always keep this much for the Midfield
  bool innerStackTaken = false;

  // ---- Red quadrant by the station ----
  tfMirror = false;
  doToggle(false, true);                          // toggle -> red
  cornerPlace(true, H_ALLIANCE_1);                // preload into the red goal
  if (timeLeft() > PARK_MS + 6000) {
    cornerGrab(true, H_GRAB_FLOOR);               // wall cup + yellow pin
    cornerPlace(true, H_ALLIANCE_2);              // ... on top of the preload
  }
  if (SKILLS_THIRD_LEVEL && timeLeft() > PARK_MS + 7000) {
    goTo(-50, -3, 135, CLAW_ANGLE, FAST, 2000, true);     // round the goal
    grabAt(STACK_IN_X, STACK_IN_Y, DIR_STACK_IN, H_GRAB_FLOOR);
    placeOn(GOAL_A_X, GOAL_A_Y, DIR_GOAL_A_OPEN, H_ALLIANCE_3);
    innerStackTaken = true;
  }
  if (timeLeft() > PARK_MS + 6000) {
    cornerGrab(false, H_GRAB_WALLPIN);            // yellow pin only
    cornerPlace(false, H_NEUTRAL_1);              // ... into the neutral goal
  }

  // ---- Cross to the other red quadrant (bottom wall) ----
  if (timeLeft() > PARK_MS + 10000) {
    goTo(-37, -9, 180, TOGGLE_MECH_ANGLE, FAST, 2500, true);
    goTo(-10, -40, 180, TOGGLE_MECH_ANGLE, FAST, 2500, true);
    tfMirror = true;              // same moves, mirrored onto the bottom wall
    doToggle(false, false);
    if (timeLeft() > PARK_MS + 6000) {
      cornerGrab(true, H_GRAB_WALLPIN);           // yellow pin only
      cornerPlace(true, H_ALLIANCE_1);            // ... into the red goal
    }
    if (timeLeft() > PARK_MS + 6500) {
      cornerGrab(false, H_GRAB_WALLPIN);          // yellow pin only
      cornerPlace(false, H_NEUTRAL_1);            // ... into the neutral goal
    }
    // The gap between the red goal and the inner stack is narrower than the
    // robot, so only go through it if that stack has already been used.
    if (innerStackTaken && timeLeft() > PARK_MS + 8500) {
      goTo(-50, 0, 180, CLAW_ANGLE, FAST, 2000, true);
      goTo(-35, -12, 180, CLAW_ANGLE, FAST, 2000, false);
      goTo(-35, -36, 180, CLAW_ANGLE, FAST, 2000, true);
      grabAt(STACK_OUT_X, STACK_OUT_Y, DIR_STACK_OUT, H_GRAB_FLOOR);
      placeOn(GOAL_A_X, GOAL_A_Y, DIR_GOAL_A_OUTER, H_ALLIANCE_2);
    }
    tfMirror = false;
  }

  // ---- Park: any part of the robot inside the Midfield diamond is 8 ----
  liftTo(H_CARRY, false);
  moveField(-13, -13, robotHeading(), FAST, 3000, true);
  moveField(-9, -9, robotHeading(), SLOW, 1500, false);
}

/*---------------------------------------------------------------------------*/
/*                              Test routine                                 */
/*---------------------------------------------------------------------------*/
// Forward 24", right 24", turn 90 clockwise, then back to where it started.
void testRoutine() {
  moveField(0, 24, 0, 60, 4000, false);
  waitMs(500);
  moveField(24, 24, 0, 60, 4000, false);
  waitMs(500);
  moveField(24, 24, 90, 60, 3000, false);
  waitMs(500);
  moveField(0, 0, 0, 60, 5000, false);
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous                                   */
/*---------------------------------------------------------------------------*/
void runSelectedRoutine() {
  int which = routine;
  int budget = (which == R_SKILLS) ? 60000 : 15000;
  if (which == R_TEST) budget = 30000;
  autonDeadline = nowMs() + budget - 250;     // stop a touch early

  if (which == R_TEST) {
    tfBlue = false;
    tfMirror = false;
    setPose(0, 0, 0);
    testRoutine();
    finishAuton();
    return;
  }

  // Skills always starts in the quadrant by the red station.
  tfBlue = (which == R_SKILLS) ? false : blueAlliance;
  tfMirror = (which == R_SKILLS) ? false : sideQuadrant;

  // Start: against the wall under the Toggle, toggle mechanism at the wall.
  setPose(fieldX(TOGGLE_X + START_WALL_DIST, TOGGLE_Y),
          fieldY(TOGGLE_X + START_WALL_DIST, TOGGLE_Y),
          fieldDir(270) - TOGGLE_MECH_ANGLE);
  // Drivers stand at their station, so "away from the driver" is known now.
  driverZero = tfBlue ? 270 : 90;

  if (which == R_SKILLS) {
    skillsRoutine();
  } else if (which != R_NONE) {
    matchRoutine(which);
  }
  finishAuton();
}

void autonomous(void) {
  testRun = false;
  runSelectedRoutine();
}

/*---------------------------------------------------------------------------*/
/*                         Brain screen selector                             */
/*---------------------------------------------------------------------------*/
void drawSelector() {
  Brain.Screen.setPenColor(color::white);

  Brain.Screen.setFillColor(blueAlliance ? color::blue : color::red);
  Brain.Screen.drawRectangle(5, 5, 230, 50);
  Brain.Screen.printAt(60, 37, true, blueAlliance ? "BLUE alliance " : "RED alliance  ");

  Brain.Screen.setFillColor(color::black);
  Brain.Screen.drawRectangle(245, 5, 230, 50);
  Brain.Screen.printAt(275, 37, true, sideQuadrant ? "Start: SIDE WALL " : "Start: BY STATION");

  Brain.Screen.drawRectangle(5, 65, 70, 55);
  Brain.Screen.printAt(32, 99, true, "<");
  Brain.Screen.drawRectangle(405, 65, 70, 55);
  Brain.Screen.printAt(435, 99, true, ">");
  Brain.Screen.drawRectangle(85, 65, 310, 55);
  Brain.Screen.printAt(100, 99, true, "%-22s", routineName(routine));

  Controller1.Screen.clearLine(2);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("%s %s", blueAlliance ? "BLU" : "RED", routineName(routine));
}

void onScreenPressed() {
  // Locked while an autonomous routine is running.
  if (testRun || (Competition.isEnabled() && Competition.isAutonomous())) {
    return;
  }
  int x = Brain.Screen.xPosition();
  int y = Brain.Screen.yPosition();
  if (y < 60) {
    if (x < 240) {
      blueAlliance = !blueAlliance;
    } else {
      sideQuadrant = !sideQuadrant;
    }
  } else if (y < 125) {
    if (x < 160) {
      routine = (routine + ROUTINE_COUNT - 1) % ROUTINE_COUNT;
    } else if (x > 320) {
      routine = (routine + 1) % ROUTINE_COUNT;
    }
  }
  selectorDirty = true;
}

// Live readout under the selector, for tuning and troubleshooting.
void showStatus() {
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.setFillColor(color::black);
  Brain.Screen.printAt(10, 150, true, "X %6.1f  Y %6.1f  Heading %6.1f     ", robotX, robotY, robotHeading());
  Brain.Screen.printAt(10, 175, true, "Lift %5.0f deg   Claw %s     ", LiftMotorA.position(degrees), clawClosed ? "closed" : "open  ");
  Brain.Screen.printAt(10, 200, true, "Side toggles: left(3) %s  right(4) %s   ",
                       ToggleLeft.installed() ? "ok" : "MISSING", ToggleRight.installed() ? "ok" : "MISSING");
  Brain.Screen.printAt(10, 225, true, "Toggle deg  L %6.0f  R %6.0f      ",
                       ToggleLeft.position(degrees), ToggleRight.position(degrees));
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous                                   */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  Lift.setStopping(hold);
  Lift.setPosition(0, degrees);       // the lift must be fully down at power-on
  ToggleLeft.setStopping(hold);
  ToggleRight.setStopping(hold);
  setClaw(true);                      // closed, so it keeps hold of the preload

  Brain.Screen.clearScreen();
  Brain.Screen.printAt(10, 30, true, "Calibrating inertial... don't move!");
  Inertial.calibrate();
  while (Inertial.isCalibrating()) {
    wait(50, msec);
  }
  Brain.Screen.clearScreen();
  setPose(0, 0, 0);
  selectorDirty = true;
}

/*---------------------------------------------------------------------------*/
/*                              User Control                                 */
/*---------------------------------------------------------------------------*/
void showDriveMode() {
  Controller1.Screen.clearLine(1);
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(fieldCentric ? "Field-centric" : "Robot-centric");
}

void toggleFieldCentric() {
  fieldCentric = !fieldCentric;
  showDriveMode();
}

// B: whichever way the robot faces now becomes "away from the driver".
void resetHeading() {
  driverZero = robotHeading();
  Controller1.rumble(".");
}

void usercontrol(void) {
  // Make sure nothing from autonomous is still running.
  autonDeadline = 0;
  stopToggles();
  showDriveMode();

  while (true) {
    updateOdometry();

    // X + Y: practise the selected auton. Never fires on a real field.
    if (Controller1.ButtonX.pressing() && Controller1.ButtonY.pressing() &&
        !Competition.isFieldControl() && !Competition.isCompetitionSwitch()) {
      testRun = true;
      runSelectedRoutine();
      testRun = false;
      autonDeadline = 0;
      selectorDirty = true;
    }

    double drive  = applyDeadband(Controller1.Axis3.position());
    double strafe = applyDeadband(Controller1.Axis4.position());
    double turn   = applyDeadband(Controller1.Axis1.position());

    if (fieldCentric) {
      // Rotate the joystick input by the robot's heading so "up" on the
      // stick always drives away from the driver.
      double headingRad = (robotHeading() - driverZero) * M_PI / 180.0;
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

    // Side toggles: hold R2 to spin both. Hold the Up arrow to spin both
    // the other way.
    if (Controller1.ButtonR2.pressing()) {
      ToggleLeft.spin(fwd, TOGGLE_SPEED, percent);
      ToggleRight.spin(fwd, TOGGLE_SPEED, percent);
    } else if (Controller1.ButtonUp.pressing()) {
      ToggleLeft.spin(reverse, TOGGLE_SPEED, percent);
      ToggleRight.spin(reverse, TOGGLE_SPEED, percent);
    } else {
      ToggleLeft.stop(hold);
      ToggleRight.stop(hold);
    }

    showStatus();

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
  Brain.Screen.pressed(onScreenPressed);

  pre_auton();

  while (true) {
    if (selectorDirty) {
      selectorDirty = false;
      drawSelector();
    }
    wait(100, msec);
  }
}
