/*----------------------------------------------------------------------------*/
/*  Team 93601V - tank drive competition code, VEX V5 Override (2026-27)      */
/*                                                                            */
/*  Ports: drive left 3 (front) + 11 (back), right 8 (front) + 20 (back),     */
/*         inertial 16, lift 1 + 10, side toggles left 2 + right 9 (5.5W),    */
/*         claw piston 3-wire A, radio any free port. Drive geared 36:60.     */
/*                                                                            */
/*  Driver: left stick forward/back, right stick turn, L1/L2 lift, R1 claw,   */
/*          hold R2 = side toggles (hold Up = the other way).                 */
/*          X + Y = test-run the selected auton (not on a field). Down = stop.*/
/*                                                                            */
/*  Before a match, tap on the brain screen: alliance, start wall (BY STATION */
/*  = the wall next to your drivers, SIDE WALL = the other wall on your side) */
/*  and routine (arrows). Skills: pick "SKILLS 60s".                          */
/*  Place the robot with its BACK flat on the wall, centred under the Toggle  */
/*  (not touching it), lift down, preload in the claw with yellow up. Keep it */
/*  still while the inertial calibrates (3 s after the program starts; tap    */
/*  CAL on the brain screen to redo it).                                      */
/*                                                                            */
/*  Tuning steps: HANDOFF.md, "Making the auton accurate". The autons have    */
/*  not been tuned on the real robot yet.                                     */
/*                                                                            */
/*  VEXcode: keep the Devices panel EMPTY and do not add "brain Brain;".      */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

using namespace vex;

/*---------------------------------------------------------------------------*/
/*                                 Devices                                   */
/*---------------------------------------------------------------------------*/
competition Competition;
controller Controller1 = controller(primary);

// Tank drive: two 11W motors per side. The right side is reversed so positive
// power drives both sides forward.
motor FrontLeft  = motor(PORT3,  ratio18_1, false);
motor BackLeft   = motor(PORT11, ratio18_1, false);
motor FrontRight = motor(PORT8,  ratio18_1, true);
motor BackRight  = motor(PORT20, ratio18_1, true);
motor_group LeftDrive  = motor_group(FrontLeft, BackLeft);
motor_group RightDrive = motor_group(FrontRight, BackRight);

inertial Inertial = inertial(PORT16);

// Cascade lift, two 11W motors. If they fight each other, flip one true/false.
motor LiftMotorA = motor(PORT1,  ratio18_1, false);
motor LiftMotorB = motor(PORT10, ratio18_1, true);
motor_group Lift = motor_group(LiftMotorA, LiftMotorB);

digital_out Claw = digital_out(Brain.ThreeWirePort.A);

// Side toggles: one 5.5W motor on each side (5.5W motors run at 200 RPM, so
// ratio18_1 is right). If one spins the wrong way, flip its true/false.
motor ToggleLeft  = motor(PORT2, ratio18_1, false);
motor ToggleRight = motor(PORT9, ratio18_1, true);

/*---------------------------------------------------------------------------*/
/*                        Settings (numbers to tune)                         */
/*---------------------------------------------------------------------------*/
// ---- Drive ----
const double WHEEL_DIAMETER   = 3.25;          // inches: MEASURE YOUR WHEELS
const double DRIVE_GEAR_RATIO = 36.0 / 60.0;   // 36 teeth on the motor, 60 on the wheel
                                               // (if it's the other way: 60.0 / 36.0)
// From "TUNE: DRIVE 48in": new FORWARD_SCALE = old x real distance / distance shown.
const double FORWARD_SCALE = 1.0;
// -1 if pushing the left stick up drives the robot backwards ("CHECK ROBOT"
// tells you). The autons only use this one, so if forward is right, so are
// their turns.
const double DRIVE_SIGN = 1;
// Driver control only: -1 if the right stick turns the wrong way for the driver.
const double TURN_SIGN = 1;
// How wide the robot turns, in inches ("CHECK ROBOT" measures it). Used to plan
// turn speeds, and to steer by the wheels if the inertial stops working.
const double TRACK_WIDTH = 12.0;
// From the hand-turn test in HANDOFF.md: 1800 / (raw change after 5 turns).
const double IMU_SCALE = 1.0;
const int    DEADBAND = 5;                     // joystick values below this count as 0
const double VOLTS_PER_PERCENT = 12.0 / 100.0; // driver control and toggles use voltage

// ---- Auton driving (percent of full speed unless it says otherwise) ----
const double FAST = 90;
const double SLOW = 40;
const double TURN_SPEED = 75;           // top speed for turns on the spot
// How hard drives and turns slow down as they arrive (per second per second):
// lower = gentler and more accurate, higher = quicker.
const double DRIVE_DECEL = 40.0;        // inches/s/s
const double TURN_DECEL  = 500.0;       // degrees/s/s
const double MIN_MOVE = 15;             // slowest drive speed near the end
const double MIN_TURN = 8;              // slowest turn speed (raise if turns stall short)
const double DRIVE_STEER_KP = 1.0;      // heading correction while driving
const double PASS_SPEED = 30;           // speed kept through a waypoint
const double SLEW = 4;                  // biggest speed change per 10 ms
const double POS_TOL = 0.5;             // inches: how close a drive must get
const double CLOSE_ENOUGH = 1.0;        // inches: already this close -> skip the move
const double HEAD_TOL = 1.0;            // degrees: how close a turn must get
const int    SETTLE_MS = 80;            // a turn must stay on target this long

// ---- Robot shape (inches). Sides are angles clockwise from the front. ----
const bool   AUTON_TOGGLE_ON_RIGHT = true;     // which side toggle the autons use
const double CLAW_SIDE   = 0;                  // the claw is on the front
const double TOGGLE_SIDE = AUTON_TOGGLE_ON_RIGHT ? 90 : 270;
const double CLAW_REACH       = 12;     // robot centre -> middle of the claw
const double START_WALL_DIST  = 9;      // robot centre -> wall at the start
// Robot centre -> wall while flipping. At least 14 so an 18" robot can turn
// there; the side toggle must reach the field Toggle from this far out.
const double TOGGLE_WALL_DIST = 14;
const double WALL_CLEARANCE   = 8;      // how far to pull off the wall after flipping
const double BACKOFF          = 5;      // line up this far back, then creep in

// ---- Side toggles ----
const int    TOGGLE_SPEED = 50;              // percent: lower = slower toggles
const double TOGGLE_FLIP_DEGREES = 360;      // motor degrees for one auton flip: TUNE
const int    TOGGLE_AUTON_TIMEOUT_MS = 2000;

// ---- Lift (motor degrees, 0 = fully down; read "Lift" on the brain screen) ----
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

// Skills: also put a third level on the first red goal.
const bool SKILLS_THIRD_LEVEL = true;

/*---------------------------------------------------------------------------*/
/*   The field: inches from the centre, +X toward the blue station, +Y "up"  */
/*   in the manual's overhead picture. Directions: 0 = +Y, 90 = +X,          */
/*   clockwise. Routines are written for RED starting BY THE STATION (left   */
/*   wall); the code mirrors them for the other three starts.                */
/*---------------------------------------------------------------------------*/
struct Spot { double x, y; };

const Spot TOGGLE    = {-72,   0};
const Spot GOAL_A    = {-48, -24};      // alliance-coloured goal
const Spot GOAL_N    = {-48,  24};      // neutral goal (starts with a yellow pin)
const Spot STACK_IN  = {-24, -24};      // clear cup + yellow pin
const Spot STACK_OUT = {-48, -48};      // clear cup + yellow pin
const Spot WALL_A    = {-69, -24};      // wall cup + yellow pin, alliance-goal side
const Spot WALL_N    = {-69,  24};      // wall cup + yellow pin, neutral-goal side
// Open spots to turn at before working in the tight corner by a wall stack.
const Spot STAGE_A   = {-56,  -6};
const Spot STAGE_N   = {-56,   6};

// Which side of a target the robot stands on (direction from target to robot).
const double DIR_GOAL_A_OPEN  = 45;     // alliance goal, from the open field
const double DIR_GOAL_A_WALL  = 320;    // alliance goal, from the wall corner
const double DIR_GOAL_A_OUTER = 135;    // alliance goal, from the outer stack
const double DIR_GOAL_N_WALL  = 220;    // neutral goal, from the wall corner
const double DIR_STACK_IN     = 315;
const double DIR_STACK_OUT    = 45;
const double DIR_WALL_A       = 50;
const double DIR_WALL_N       = 130;
// Robot centre -> wall stack while lining up to grab it.
const double CORNER_LINEUP = CLAW_REACH + BACKOFF + 6;

/*---------------------------------------------------------------------------*/
/*                          Routines and state                               */
/*---------------------------------------------------------------------------*/
enum Routine {
  R_NONE, R_TOGGLE, R_PRELOAD, R_TOGGLE_PRELOAD, R_FIELD_STACK, R_WALL_STACK,
  R_FULL, R_SKILLS, R_TEST, R_CHECK, R_TUNE_DRIVE, R_TUNE_TURN, ROUTINE_COUNT
};
const char *ROUTINE_NAMES[ROUTINE_COUNT] = {
  "DO NOTHING", "TOGGLE ONLY", "PRELOAD ONLY", "TOGGLE + PRELOAD",
  "TOG+PRE+FIELD STACK", "TOG+PRE+WALL STACK", "FULL QUADRANT", "SKILLS 60s",
  "TEST (24in square)", "CHECK ROBOT", "TUNE: DRIVE 48in", "TUNE: TURN 90 x4"
};

bool blueAlliance = false;              // picked on the brain screen
bool sideQuadrant = false;              // false = start by your alliance station
int  routine = R_TOGGLE;
bool selectorDirty = true;              // redraw the selector
bool imuReady = false;                  // false while the inertial is calibrating
bool imuMovedWarning = false;           // the heading crept right after calibrating
volatile bool calRequested = false;     // CAL tapped on the brain screen
bool clawClosed = false;
char resultA[64] = "";                  // two result lines at the bottom of the
char resultB[64] = "";                  // brain screen (CHECK, TUNE, safety stop)

int nowMs() { return (int)Brain.Timer.time(msec); }

int applyDeadband(int value) { return (std::abs(value) < DEADBAND) ? 0 : value; }

double wrap180(double a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

/*---------------------------------------------------------------------------*/
/*                                  Drive                                    */
/*---------------------------------------------------------------------------*/
// Sends power (-100..100 percent) to each side, scaled down together if either
// is over 100%. voltage = true for driver control (12 V = 100%); false for the
// autons, which use the motors' own speed control (the same on any battery).
void setDrive(double left, double right, bool voltage) {
  double maxPower = std::fmax(std::fabs(left), std::fabs(right));
  if (maxPower > 100) {
    left  = left  / maxPower * 100;
    right = right / maxPower * 100;
  }
  if (voltage) {
    LeftDrive.spin(fwd, left * VOLTS_PER_PERCENT, volt);
    RightDrive.spin(fwd, right * VOLTS_PER_PERCENT, volt);
  } else {
    LeftDrive.spin(fwd, left, percent);
    RightDrive.spin(fwd, right, percent);
  }
}

// Driver control. forward and turn (clockwise) are -100..100.
void tankDrive(double forward, double turn) {
  forward = DRIVE_SIGN * forward;
  turn    = TURN_SIGN  * turn;
  setDrive(forward + turn, forward - turn, true);
}

// Autons. Same, but closed-loop, and only DRIVE_SIGN is used.
void autonDrive(double forward, double turn) {
  setDrive(DRIVE_SIGN * (forward + turn), DRIVE_SIGN * (forward - turn), false);
}

// Speed a passThrough driveTo was still rolling at when it handed over, so the
// next move carries on from there instead of from a standstill.
double carriedSpeed = 0;
bool   carriedBackwards = false;

void stopDrive() {
  LeftDrive.stop(brake);
  RightDrive.stop(brake);
  carriedSpeed = 0;
}

/*---------------------------------------------------------------------------*/
/*     Odometry: where the robot is, from the wheel encoders + inertial      */
/*---------------------------------------------------------------------------*/
double robotX = 0, robotY = 0;          // inches, field coordinates
double fusedHeading = 0;                // degrees, clockwise, not wrapped
double prevLeft = 0, prevRight = 0, prevImu = 0;
bool   imuWasReadable = false;

// The inertial is plugged in and not calibrating.
bool imuReadable() { return Inertial.installed() && !Inertial.isCalibrating(); }

// Inertial rotation in degrees (clockwise, not wrapped), with IMU_SCALE.
double imuRotation() { return Inertial.rotation(degrees) * IMU_SCALE; }

double robotHeading() { return fusedHeading; }

double leftDeg()  { return (FrontLeft.position(degrees)  + BackLeft.position(degrees))  / 2.0; }
double rightDeg() { return (FrontRight.position(degrees) + BackRight.position(degrees)) / 2.0; }

// Inches the robot moves per degree the drive motors turn.
double inchesPerMotorDegree() {
  return M_PI * WHEEL_DIAMETER / 360.0 * DRIVE_GEAR_RATIO * FORWARD_SCALE;
}

void setPose(double x, double y, double heading) {
  robotX = x;
  robotY = y;
  fusedHeading = heading;
  prevLeft = leftDeg();
  prevRight = rightDeg();
  imuWasReadable = imuReadable();
  if (imuWasReadable) prevImu = imuRotation();
}

// Distance comes from the drive encoders. Turning comes from the inertial, or
// from the wheels while the inertial is unplugged or restarting (so a loose
// cable doesn't send the robot off in a random direction).
void updateOdometry() {
  double l = leftDeg();
  double r = rightDeg();
  double dL = (l - prevLeft)  * inchesPerMotorDegree() * DRIVE_SIGN;   // inches, left side
  double dR = (r - prevRight) * inchesPerMotorDegree() * DRIVE_SIGN;   // inches, right side
  prevLeft = l;
  prevRight = r;

  double turned = (dL - dR) / TRACK_WIDTH * 180.0 / M_PI;              // from the wheels
  bool readable = imuReadable();
  if (readable) {
    double now = imuRotation();
    if (imuWasReadable && std::fabs(now - prevImu) < 90) {
      turned = now - prevImu;           // normal case: trust the inertial
    }                                   // else it just came back: start from here
    prevImu = now;
  }
  imuWasReadable = readable;

  double before = fusedHeading;
  fusedHeading += turned;
  double mid = (before + fusedHeading) / 2.0 * M_PI / 180.0;
  double ahead = (dL + dR) / 2.0;
  robotX += ahead * std::sin(mid);
  robotY += ahead * std::cos(mid);
}

/*---------------------------------------------------------------------------*/
/*                         Claw and side toggles                             */
/*---------------------------------------------------------------------------*/
void setClaw(bool closed) {
  clawClosed = closed;
  Claw.set(closed);
}

void toggleClaw() { setClaw(!clawClosed); }

void spinToggles(directionType dir) {
  ToggleLeft.spin(dir, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
  ToggleRight.spin(dir, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
}

void stopToggles() {
  ToggleLeft.stop(hold);
  ToggleRight.stop(hold);
}

/*---------------------------------------------------------------------------*/
/*                      Auton basics: turn and drive                         */
/*---------------------------------------------------------------------------*/
int  autonDeadline = 0;     // every step gives up at this time
bool testRun = false;       // started with X + Y, not by the field
bool tfBlue = false;        // how the routine is mirrored onto the field
bool tfMirror = false;

int timeLeft() { return autonDeadline - nowMs(); }

// Down arrow stops an X + Y test-run.
bool downPressed() { return testRun && Controller1.ButtonDown.pressing(); }

void checkAbort() {
  if (downPressed()) autonDeadline = 0;
}

// Waits while keeping the position up to date.
void waitMs(int ms) {
  int end = nowMs() + ms;
  do {
    updateOdometry();
    checkAbort();
    wait(10, msec);
  } while (nowMs() < end && timeLeft() > 0);
}

// Safety stop: the robot turned the opposite way to what it was told, so
// DRIVE_SIGN or a drive motor's true/false is wrong. Stops the whole auton
// instead of spinning out of control.
void wrongWayStop() {
  stopDrive();
  autonDeadline = 0;
  snprintf(resultA, sizeof(resultA), "STOPPED: robot turned the wrong way.");
  snprintf(resultB, sizeof(resultB), "Run CHECK ROBOT to see what to fix.");
  Controller1.rumble("...");
}

// Turn on the spot to a field heading.
void turnTo(double th, double maxSpeed, int timeoutMs) {
  if (timeLeft() <= 0) return;
  updateOdometry();
  int end = nowMs() + timeoutMs;
  double startHeading = robotHeading();
  double startErr = wrap180(th - startHeading);
  // Turning speed (degrees per second) at 100%.
  double topTurnRate = 2.0 * (200.0 * 360.0 / 60.0 * inchesPerMotorDegree()) / TRACK_WIDTH * 180.0 / M_PI;
  int settled = 0;
  while (nowMs() < end && timeLeft() > 0) {
    updateOdometry();
    checkAbort();
    double err = wrap180(th - robotHeading());
    // How far it has turned towards the target (negative = the wrong way).
    double progress = (robotHeading() - startHeading) * (startErr >= 0 ? 1 : -1);
    if (std::fabs(startErr) > 5 && progress < -25) {
      wrongWayStop();
      return;
    }
    if (std::fabs(err) < HEAD_TOL) {
      // On target: hold still, and finish once it has stayed there.
      settled += 10;
      if (settled >= SETTLE_MS) break;
      autonDrive(0, 0);
      wait(10, msec);
      continue;
    }
    settled = 0;
    // Stopping curve: fast while far off, then slowing evenly (TURN_DECEL).
    double turn = std::sqrt(2.0 * TURN_DECEL * std::fabs(err)) / topTurnRate * 100.0;
    if (turn > maxSpeed) turn = maxSpeed;
    if (turn < MIN_TURN) turn = MIN_TURN;
    autonDrive(0, err > 0 ? turn : -turn);
    wait(10, msec);
  }
  stopDrive();
}

// Drive to a field point: turn to face it, then drive with heading correction.
// Drives backwards if the point is behind, so backing away needs no U-turn.
// passThrough = don't stop at the end, just get close and carry on.
// timeoutMs is for the driving part; the turn first gets its own time.
void driveTo(double tx, double ty, double maxSpeed, int timeoutMs, bool passThrough) {
  if (timeLeft() <= 0) return;
  updateOdometry();
  double posTol = passThrough ? 3.0 : POS_TOL;

  // Already (nearly) there: don't turn round just to creep an inch.
  double ex = tx - robotX, ey = ty - robotY;
  if (std::sqrt(ex * ex + ey * ey) < (passThrough ? posTol : CLOSE_ENOUGH)) return;
  double err = wrap180(std::atan2(ex, ey) * 180.0 / M_PI - robotHeading());
  bool backwards = std::fabs(err) > 90;
  if (backwards) err = wrap180(err + 180);
  if (std::fabs(err) > 15) {
    turnTo(robotHeading() + err, TURN_SPEED, 1500);
  }
  int end = nowMs() + timeoutMs;

  // Keep rolling from a passThrough move that was going the same way
  // (turnTo above calls stopDrive, which clears carriedSpeed).
  double lastSpeed = (carriedSpeed > 0 && carriedBackwards == backwards) ? carriedSpeed : 0;
  carriedSpeed = 0;
  while (nowMs() < end && timeLeft() > 0) {
    updateOdometry();
    checkAbort();
    ex = tx - robotX;
    ey = ty - robotY;
    double dist = std::sqrt(ex * ex + ey * ey);
    double aim = std::atan2(ex, ey) * 180.0 / M_PI + (backwards ? 180 : 0);
    err = wrap180(aim - robotHeading());
    double along = dist * std::cos(err * M_PI / 180.0);   // distance left along the robot
    if (along < posTol) break;                             // arrived (or just passed it)
    // Knocked or slipped far off line: stop this move; the next one re-aims.
    if (dist > 6 && std::fabs(err) > 60) break;

    // Stopping curve: full speed while far away, then slowing evenly (DRIVE_DECEL).
    double topSpeed = 200.0 * 360.0 / 60.0 * inchesPerMotorDegree();   // inches/s at 100%
    double speed = std::sqrt(2.0 * DRIVE_DECEL * along) / topSpeed * 100.0;
    if (passThrough && speed < PASS_SPEED) speed = PASS_SPEED;   // keep rolling
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed > lastSpeed + SLEW) speed = lastSpeed + SLEW;
    if (speed < MIN_MOVE) speed = MIN_MOVE;
    lastSpeed = speed;

    // Steer toward the point, but not in the last few inches where the angle
    // to it swings about.
    double turn = (dist > 6) ? DRIVE_STEER_KP * err : 0;
    if (turn > 40) turn = 40;
    if (turn < -40) turn = -40;

    autonDrive(backwards ? -speed : speed, turn);
    wait(10, msec);
  }
  if (passThrough) {
    carriedSpeed = lastSpeed;
    carriedBackwards = backwards;
  } else {
    stopDrive();
  }
}

/*---------------------------------------------------------------------------*/
/*      Moves for routines (routine spots, mirrored onto the real field)     */
/*---------------------------------------------------------------------------*/
Spot onField(Spot s) {
  double x = tfMirror ? s.y : s.x;
  double y = tfMirror ? s.x : s.y;
  if (tfBlue) {
    x = -x;
    y = -y;
  }
  return {x, y};
}

double fieldDir(double dir) {
  if (tfMirror) dir = 90 - dir;
  if (tfBlue) dir += 180;
  return dir;
}

// Drive to a spot and stop there.
void go(Spot s, double speed, int timeoutMs) {
  Spot f = onField(s);
  driveTo(f.x, f.y, speed, timeoutMs, false);
}

// Drive through a spot without stopping (a waypoint).
void via(Spot s, double speed = FAST, int timeoutMs = 2500) {
  Spot f = onField(s);
  driveTo(f.x, f.y, speed, timeoutMs, true);
}

// Turn so one side of the robot (CLAW_SIDE or TOGGLE_SIDE) points in a direction.
void face(double dir, double side = CLAW_SIDE) {
  turnTo(fieldDir(dir) - side, TURN_SPEED, 1500);
}

// Line up on a target: stand `reach` inches from it on its fromDir side, with
// `side` of the robot pointing at it. passThrough = keep going, don't stop.
void lineUp(Spot target, double fromDir, double reach, double side,
            double speed, int timeoutMs, bool passThrough = false) {
  double r = fromDir * M_PI / 180.0;
  Spot spot = {target.x + reach * std::sin(r), target.y + reach * std::cos(r)};
  if (passThrough) {
    via(spot, speed, timeoutMs);
  } else {
    go(spot, speed, timeoutMs);
    face(fromDir + 180, side);
  }
}

/*---------------------------------------------------------------------------*/
/*                Lift, claw and toggle actions for routines                 */
/*---------------------------------------------------------------------------*/
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

// Spin the side toggle against the wall by one flip: a fixed amount, so start
// every run with the field Toggle in the same position and tune
// TOGGLE_FLIP_DEGREES for it. reverseIt = the other way (lands on blue).
void flipToggle(bool reverseIt) {
  if (timeLeft() <= 0) return;
  motor &side = AUTON_TOGGLE_ON_RIGHT ? ToggleRight : ToggleLeft;
  double startPos = side.position(degrees);
  int start = nowMs();
  side.spin(reverseIt ? reverse : fwd, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
  while (std::fabs(side.position(degrees) - startPos) < TOGGLE_FLIP_DEGREES &&
         nowMs() - start < TOGGLE_AUTON_TIMEOUT_MS && timeLeft() > 0) {
    waitMs(10);
  }
  side.stop(hold);
}

// Flip this quadrant's Toggle: stand TOGGLE_WALL_DIST off the wall with the
// toggle side facing it, flip, then pull away (a Toggle only counts when no
// robot touches it).
void doToggle(bool wantBlue, bool alreadyAtWall) {
  if (!alreadyAtWall) {
    lineUp(TOGGLE, 90, TOGGLE_WALL_DIST + WALL_CLEARANCE, TOGGLE_SIDE, FAST, 3000, true);
  }
  lineUp(TOGGLE, 90, TOGGLE_WALL_DIST, TOGGLE_SIDE, SLOW, 1500);
  flipToggle(wantBlue);
  lineUp(TOGGLE, 90, TOGGLE_WALL_DIST + WALL_CLEARANCE, TOGGLE_SIDE, FAST, 1200, true);
}

// Pick something up with the claw.
void grabAt(Spot target, double fromDir, double height) {
  if (timeLeft() <= 0) return;
  autonClaw(false);
  liftTo(height, false);
  lineUp(target, fromDir, CLAW_REACH + BACKOFF, CLAW_SIDE, FAST, 2500);
  liftTo(height, true);
  lineUp(target, fromDir, CLAW_REACH, CLAW_SIDE, SLOW, 1500);
  autonClaw(true);
  waitMs(GRAB_MS);
  liftTo(height + H_LIFT_OFF, true);
  lineUp(target, fromDir, CLAW_REACH + BACKOFF, CLAW_SIDE, SLOW, 1000);   // back off
}

// Put what the claw holds onto a goal (or onto the stack on it).
void placeOn(Spot goal, double fromDir, double height) {
  if (timeLeft() <= 0) return;
  liftTo(height + H_LIFT_OFF, false);
  lineUp(goal, fromDir, CLAW_REACH + BACKOFF, CLAW_SIDE, FAST, 2500);
  liftTo(height + H_LIFT_OFF, true);
  lineUp(goal, fromDir, CLAW_REACH, CLAW_SIDE, SLOW, 1500);
  liftTo(height, true);
  autonClaw(false);
  waitMs(RELEASE_MS);
  lineUp(goal, fromDir, CLAW_REACH + BACKOFF, CLAW_SIDE, SLOW, 1000);       // back off
  liftTo(H_CARRY, false);
}

// The wall stack and the goal beside it are only 21" apart, so the robot must
// not turn on the spot right next to them. Grabbing: line up further out on the
// claw's line, drive straight in and straight back out. Placing: turn at an
// open "stage" spot first. allianceSide = the pair on the alliance-goal side.
void cornerGrab(bool allianceSide, double height) {
  if (timeLeft() <= 0) return;
  Spot stack = allianceSide ? WALL_A : WALL_N;
  double dir = allianceSide ? DIR_WALL_A : DIR_WALL_N;
  lineUp(stack, dir, CORNER_LINEUP, CLAW_SIDE, FAST, 2500);
  grabAt(stack, dir, height);
  lineUp(stack, dir, CORNER_LINEUP, CLAW_SIDE, FAST, 1500);   // straight back out
}

void cornerPlace(bool allianceSide, double height) {
  if (timeLeft() <= 0) return;
  Spot stage = allianceSide ? STAGE_A : STAGE_N;
  double dir = allianceSide ? DIR_GOAL_A_WALL : DIR_GOAL_N_WALL;
  go(stage, FAST, 2500);
  face(dir + 180);
  placeOn(allianceSide ? GOAL_A : GOAL_N, dir, height);
  go(stage, FAST, 1500);
}

void finishAuton() {
  stopDrive();
  stopToggles();
  // Let a lift move that is still going finish, instead of freezing the lift.
  int end = nowMs() + 1000;
  while (!Lift.isDone() && nowMs() < end && !downPressed()) {
    wait(10, msec);
  }
  Lift.stop(hold);
}

/*---------------------------------------------------------------------------*/
/*                         15 second match routines                          */
/*---------------------------------------------------------------------------*/
// Each routine does the steps up to its own. A step is skipped if there isn't
// time to finish it, so a slow run ends early instead of half-way through.
void matchRoutine(int which) {
  // 1. Toggle: makes every visible yellow pin half in this quadrant ours (the
  //    neutral goal already holds a yellow pin, so this alone is 10 points).
  if (which == R_PRELOAD) {
    lineUp(TOGGLE, 90, TOGGLE_WALL_DIST + WALL_CLEARANCE, TOGGLE_SIDE, FAST, 1200, true);
  } else {
    doToggle(tfBlue, true);
  }
  if (which == R_TOGGLE) return;

  // 2. Preload into the alliance goal.
  if (timeLeft() < 4000) return;
  if (which == R_WALL_STACK) {
    cornerPlace(true, H_ALLIANCE_1);
  } else {
    placeOn(GOAL_A, DIR_GOAL_A_OPEN, H_ALLIANCE_1);
  }
  if (which == R_PRELOAD || which == R_TOGGLE_PRELOAD) return;

  // 3. A cup + yellow pin on top of the preload: two scored halves.
  if (timeLeft() < 6000) return;
  if (which == R_WALL_STACK) {
    cornerGrab(true, H_GRAB_FLOOR);
    cornerPlace(true, H_ALLIANCE_2);
    return;
  }
  grabAt(STACK_IN, DIR_STACK_IN, H_GRAB_FLOOR);
  placeOn(GOAL_A, DIR_GOAL_A_OPEN, H_ALLIANCE_2);
  if (which == R_FIELD_STACK) return;

  // 4. FULL only: the wall cup + yellow pin onto the neutral goal's pin.
  if (timeLeft() < 6500) return;
  cornerGrab(false, H_GRAB_FLOOR);
  cornerPlace(false, H_NEUTRAL_2);
}

/*---------------------------------------------------------------------------*/
/*                        60 second skills routine                           */
/*---------------------------------------------------------------------------*/
// Starts in the quadrant by the red station (rule RSC2). Works the two red
// quadrants, then parks in the Midfield. Goals start empty in skills.
void skillsRoutine() {
  const int PARK_MS = 6500;       // always keep this much for the Midfield
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
    via({-50, -3}, FAST, 2000);                   // round the goal
    grabAt(STACK_IN, DIR_STACK_IN, H_GRAB_FLOOR);
    placeOn(GOAL_A, DIR_GOAL_A_OPEN, H_ALLIANCE_3);
    innerStackTaken = true;
  }
  if (timeLeft() > PARK_MS + 6000) {
    cornerGrab(false, H_GRAB_WALLPIN);            // yellow pin only
    cornerPlace(false, H_NEUTRAL_1);              // ... into the neutral goal
  }

  // ---- Cross to the other red quadrant (bottom wall) ----
  if (timeLeft() > PARK_MS + 10000) {
    via({-37, -9});
    if (!innerStackTaken) {
      via({-8, -9});              // go round the inner stack while it's there
    }
    via({-10, -40});
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
      via({-50, 0}, FAST, 2000);
      go({-35, -12}, FAST, 2000);
      face(180);
      via({-35, -36}, FAST, 2000);
      grabAt(STACK_OUT, DIR_STACK_OUT, H_GRAB_FLOOR);
      placeOn(GOAL_A, DIR_GOAL_A_OUTER, H_ALLIANCE_2);
    }
    tfMirror = false;
  }

  // ---- Park: any part of the robot inside the Midfield diamond is 8 ----
  liftTo(H_CARRY, false);
  via({-36, -6});                 // waypoint, to stay clear of the inner stack
  go({-13, -11}, FAST, 3000);
}

/*---------------------------------------------------------------------------*/
/*                     Test, check and tuning routines                       */
/*---------------------------------------------------------------------------*/
// TEST: forward 24", turn right, forward 24", then back to the start.
void testRoutine() {
  go({0, 24}, 60, 4000);
  face(0);
  waitMs(500);
  turnTo(90, TURN_SPEED, 2000);
  waitMs(500);
  go({24, 24}, 60, 4000);
  face(90);
  waitMs(500);
  go({0, 0}, 60, 6000);
  face(0);
}

// CHECK ROBOT: spins a little clockwise and back, and uses the inertial to
// check DRIVE_SIGN and the drive motors' true/false. Also measures TRACK_WIDTH.
void checkRoutine() {
  if (!imuReadable()) {
    snprintf(resultA, sizeof(resultA), "CHECK: no inertial on port 16, or it is");
    snprintf(resultB, sizeof(resultB), "still calibrating. Plug in / tap CAL.");
    return;
  }
  double imu0 = imuRotation();
  double l0 = leftDeg();
  double r0 = rightDeg();
  autonDrive(0, 25);                  // ask for a clockwise spin
  waitMs(600);
  stopDrive();
  waitMs(400);                        // let it stop before measuring
  double imuTurn = imuRotation() - imu0;
  double dL = (leftDeg() - l0) * inchesPerMotorDegree();    // motor direction
  double dR = (rightDeg() - r0) * inchesPerMotorDegree();

  autonDrive(0, -25);                 // spin back to roughly where it was
  waitMs(600);
  stopDrive();
  waitMs(300);

  if (std::fabs(dL) < 0.5 && std::fabs(dR) < 0.5) {
    snprintf(resultA, sizeof(resultA), "CHECK: drive motors did not move.");
    snprintf(resultB, sizeof(resultB), "Check drive ports 3, 11, 8, 20.");
  } else if (std::fabs(imuTurn) < 5) {
    snprintf(resultA, sizeof(resultA), "CHECK: robot did not turn. One drive side");
    snprintf(resultB, sizeof(resultB), "is reversed wrong (motor true/false).");
  } else {
    int needed = ((imuTurn > 0) == (dL - dR > 0)) ? 1 : -1;
    double track = std::fabs(dL - dR) / (std::fabs(imuTurn) * M_PI / 180.0);
    if (needed != (int)DRIVE_SIGN) {
      snprintf(resultA, sizeof(resultA), "CHECK: DRIVE_SIGN is WRONG, set it to %d", needed);
    } else {
      snprintf(resultA, sizeof(resultA), "CHECK OK: turned %.0f deg, DRIVE_SIGN right", imuTurn);
    }
    snprintf(resultB, sizeof(resultB), "Set TRACK_WIDTH = %.1f (now %.1f)", track, TRACK_WIDTH);
  }
}

// TUNE: DRIVE 48in. Measure how far it really went to set FORWARD_SCALE.
void tuneDriveRoutine() {
  go({0, 48}, 50, 6000);
  waitMs(500);
  if (timeLeft() <= 0) return;
  snprintf(resultA, sizeof(resultA), "Code says it drove %.1f in (%.1f sideways)", robotY, robotX);
  snprintf(resultB, sizeof(resultB), "Measure real D: FORWARD_SCALE x D / %.1f", robotY);
}

// TUNE: TURN 90 x4. A full circle in right angles; it should end facing where
// it started. Shows the worst error the inertial saw.
void tuneTurnRoutine() {
  double worst = 0;
  for (int i = 1; i <= 4 && timeLeft() > 0; i++) {
    turnTo(90.0 * i, TURN_SPEED, 2000);
    waitMs(300);
    double e = 90.0 * i - robotHeading();
    if (std::fabs(e) > std::fabs(worst)) worst = e;
  }
  if (timeLeft() <= 0 && resultA[0] == 'S') return;   // a safety stop is showing
  snprintf(resultA, sizeof(resultA), "Turns: worst error %.1f deg", worst);
  snprintf(resultB, sizeof(resultB), "Not facing start? Set IMU_SCALE");
}

/*---------------------------------------------------------------------------*/
/*                               Autonomous                                  */
/*---------------------------------------------------------------------------*/
void runSelectedRoutine() {
  int which = routine;
  bool tool = (which >= R_TEST);          // TEST, CHECK and the TUNE routines
  int budget = tool ? 30000 : (which == R_SKILLS ? 60000 : 15000);

  // Just switched on? Wait for the inertial to finish calibrating.
  while (!imuReady) {
    wait(10, msec);
  }
  autonDeadline = nowMs() + budget - 250;     // stop a touch early
  resultA[0] = 0;
  resultB[0] = 0;
  if (!Inertial.installed()) {
    snprintf(resultA, sizeof(resultA), "NO INERTIAL: steering by the wheels only");
    Controller1.rumble("---");
  }

  if (tool) {
    tfBlue = false;
    tfMirror = false;
    setPose(0, 0, 0);
    if (which == R_TEST)       testRoutine();
    if (which == R_CHECK)      checkRoutine();
    if (which == R_TUNE_DRIVE) tuneDriveRoutine();
    if (which == R_TUNE_TURN)  tuneTurnRoutine();
  } else {
    // Skills always starts in the quadrant by the red station.
    tfBlue   = (which == R_SKILLS) ? false : blueAlliance;
    tfMirror = (which == R_SKILLS) ? false : sideQuadrant;
    // Start: back against the wall under the Toggle, facing into the field.
    Spot start = onField({TOGGLE.x + START_WALL_DIST, TOGGLE.y});
    setPose(start.x, start.y, fieldDir(90));
    if (which == R_SKILLS) {
      skillsRoutine();
    } else if (which != R_NONE) {
      matchRoutine(which);
    }
  }
  finishAuton();
}

void autonomous(void) {
  testRun = false;
  runSelectedRoutine();
}

/*---------------------------------------------------------------------------*/
/*                          Brain screen selector                            */
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
  Brain.Screen.printAt(100, 99, true, "%-22s", ROUTINE_NAMES[routine]);

  // CAL: redo the inertial calibration (robot must be still).
  Brain.Screen.drawRectangle(400, 126, 75, 42);
  Brain.Screen.printAt(420, 153, true, "CAL");

  Controller1.Screen.clearLine(2);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("%s %s", blueAlliance ? "BLU" : "RED", ROUTINE_NAMES[routine]);
}

void onScreenPressed() {
  // Locked while an auton or an X+Y test-run is running. (testRun only counts
  // in driver control: if a test-run was cut off by plugging into the field,
  // the selector must still work while disabled.)
  if (Competition.isEnabled() &&
      (Competition.isAutonomous() || (testRun && Competition.isDriverControl()))) {
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
  } else if (x >= 400 && y <= 170) {
    calRequested = true;                // done by the main loop, not here
  }
  selectorDirty = true;
}

// Live readout under the selector, shown all the time (also before a match).
void showStatus() {
  Brain.Screen.setFillColor(color::black);
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.printAt(10, 140, true, "X %6.1f Y %6.1f H %6.1f  ", robotX, robotY, robotHeading());

  // Inertial status (the CAL box is to the right, so keep this line short).
  if (!imuReady) {
    Brain.Screen.setPenColor(color::yellow);
    Brain.Screen.printAt(10, 162, true, "IMU calibrating, DON'T MOVE ");
  } else if (!Inertial.installed()) {
    Brain.Screen.setPenColor(color::red);
    Brain.Screen.printAt(10, 162, true, "IMU NOT FOUND (port 16)     ");
  } else if (imuMovedWarning) {
    Brain.Screen.setPenColor(color::yellow);
    Brain.Screen.printAt(10, 162, true, "IMU moved? Keep still, CAL  ");
  } else {
    Brain.Screen.printAt(10, 162, true, "IMU OK  raw %8.1f      ", Inertial.rotation(degrees));
  }
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.printAt(10, 184, true, "Lift %5.0f deg   Claw %s   ", LiftMotorA.position(degrees), clawClosed ? "closed" : "open  ");
  Brain.Screen.printAt(10, 206, true, "%-46s", resultA);
  Brain.Screen.printAt(10, 228, true, "%-46s", resultB);
}

/*---------------------------------------------------------------------------*/
/*                             Pre-autonomous                                */
/*---------------------------------------------------------------------------*/
// Calibrates the inertial (2-3 seconds; the robot must be completely still)
// and checks it worked. Runs at power-on and when CAL is tapped.
void calibrateInertial() {
  imuReady = false;
  imuMovedWarning = false;
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(color::black);
  if (!Inertial.installed()) {
    Brain.Screen.setPenColor(color::red);
    Brain.Screen.printAt(10, 30, true, "NO INERTIAL FOUND ON PORT 16!");
    Brain.Screen.printAt(10, 55, true, "Autons will steer by the wheels only.");
    Brain.Screen.setPenColor(color::white);
    Controller1.rumble("---");
    wait(1500, msec);
  } else {
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.printAt(10, 30, true, "Calibrating inertial...");
    Brain.Screen.printAt(10, 55, true, "DON'T TOUCH OR MOVE THE ROBOT");
    Inertial.calibrate();
    wait(200, msec);                    // isCalibrating() can take a moment to turn on
    int start = nowMs();
    while (Inertial.isCalibrating() && nowMs() - start < 6000) {
      wait(20, msec);
    }
    // A good calibration holds still. If the heading creeps while the robot
    // sits there, it was probably moved during calibration.
    double r0 = Inertial.rotation(degrees);
    wait(500, msec);
    if (std::fabs(Inertial.rotation(degrees) - r0) > 0.3) {
      imuMovedWarning = true;
      Controller1.rumble("--");
    }
  }
  Brain.Screen.clearScreen();
  setPose(0, 0, 0);
  selectorDirty = true;
  imuReady = true;
}

void pre_auton(void) {
  Lift.setStopping(hold);
  Lift.setPosition(0, degrees);         // the lift must be fully down at power-on
  ToggleLeft.setStopping(hold);
  ToggleRight.setStopping(hold);
  setClaw(true);                        // closed, so it keeps hold of the preload
  calibrateInertial();
}

/*---------------------------------------------------------------------------*/
/*                              Driver control                               */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  // Make sure nothing from an auton or a cut-off test-run is still running.
  autonDeadline = 0;
  testRun = false;
  selectorDirty = true;
  stopToggles();
  Controller1.Screen.clearLine(1);
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("93601V tank drive");

  // Without a field or switch, driver control starts while the inertial is
  // still calibrating. Moving the robot then would spoil the calibration.
  while (!imuReady) {
    wait(20, msec);
  }

  while (true) {
    updateOdometry();

    // X + Y: practise the selected auton. Never on a real field.
    if (imuReady && Controller1.ButtonX.pressing() && Controller1.ButtonY.pressing() &&
        !Competition.isFieldControl() && !Competition.isCompetitionSwitch()) {
      testRun = true;
      runSelectedRoutine();
      testRun = false;
      autonDeadline = 0;
      selectorDirty = true;
      // Wait for X and Y to be let go, so one press runs it once and an abort
      // with Down stays aborted.
      while (Controller1.ButtonX.pressing() || Controller1.ButtonY.pressing()) {
        stopDrive();
        wait(20, msec);
      }
    }

    // Arcade: left stick up/down drives, right stick left/right turns.
    double forward = applyDeadband(Controller1.Axis3.position());
    double turn    = applyDeadband(Controller1.Axis1.position());
    if (forward == 0 && turn == 0) {
      stopDrive();
    } else {
      tankDrive(forward, turn);
    }

    // Lift: hold L1 to go up, L2 to go down; holds position when released.
    if (Controller1.ButtonL1.pressing()) {
      Lift.spin(fwd, LIFT_SPEED * VOLTS_PER_PERCENT, volt);
    } else if (Controller1.ButtonL2.pressing()) {
      Lift.spin(reverse, LIFT_SPEED * VOLTS_PER_PERCENT, volt);
    } else {
      Lift.stop(hold);
    }

    // Side toggles: hold R2 to spin both, hold Up to spin both the other way.
    if (Controller1.ButtonR2.pressing()) {
      spinToggles(fwd);
    } else if (Controller1.ButtonUp.pressing()) {
      spinToggles(reverse);
    } else {
      stopToggles();
    }

    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Registered here, not in usercontrol, so each press only fires once even
  // when the field or switch restarts driver control.
  Controller1.ButtonR1.pressed(toggleClaw);
  Brain.Screen.pressed(onScreenPressed);

  pre_auton();

  // All brain-screen drawing happens here, in one place.
  while (true) {
    if (calRequested) {
      calRequested = false;
      // Never in the middle of an auton or an X+Y test-run.
      bool running = testRun || (Competition.isEnabled() && Competition.isAutonomous());
      if (!running) calibrateInertial();
    }
    if (selectorDirty) {
      selectorDirty = false;
      drawSelector();
    }
    showStatus();
    wait(100, msec);
  }
}
