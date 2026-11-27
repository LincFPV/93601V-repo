/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Team 93601V - Tank drive competition code for VEX V5 Override        */
/*                                                                            */
/*    Ports (same as TankDrive.cpp). X-drive version: XDriveComp.cpp         */
/*      Drive (4 x 11W = 44W): left 3 (front) + 11 (back),                    */
/*                             right 8 (front) + 20 (back)                    */
/*      Inertial 16, Radio: any free port   Drive geared 36:60 (motor:wheel)  */
/*      Lift 1, 10        Side toggles: left 2, right 9 (5.5W motors)         */
/*      Claw piston: 3-wire port A                                            */
/*      Drive, lift and side toggles run on voltage (12 V = full power).      */
/*                                                                            */
/*    Driver controls:                                                        */
/*      Left stick up/down = forward / back    Right stick X = turn           */
/*      L1 / L2 = lift up / down         R1 = claw                            */
/*      R2 (hold) = spin both side toggles                                    */
/*      Up arrow (hold) = spin both side toggles the other way                */
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
/*      BACK of the robot flat against the wall, centred under the Toggle,    */
/*      facing into the field, lift down. Nothing may touch the Toggle at     */
/*      the start (rule SG1). Preload in the claw, yellow end UP. Power on    */
/*      with the lift down and don't move the robot while the inertial        */
/*      calibrates.                                                           */
/*                                                                            */
/*    TUNING ORDER (nothing below has been run on the real robot yet):        */
/*      1. Push the left stick up: the robot must drive forward. If not,      */
/*         set DRIVE_SIGN to -1. Push the right stick right: it must turn     */
/*         clockwise. If not, set TURN_SIGN to -1. The autons need both       */
/*         right or they will spin in circles.                                */
/*      2. Run routine "TEST" with X+Y: forward 24", turn right, forward      */
/*         24", then back to the start facing forward. If the distance is     */
/*         wrong fix WHEEL_DIAMETER (or FORWARD_SCALE).                       */
/*      3. Drive the lift by hand and read "Lift" on the brain screen to      */
/*         fill in every H_... height.                                        */
/*      4. Measure CLAW_REACH and START_WALL_DIST, set AUTON_TOGGLE_ON_RIGHT, */
/*         set TOGGLE_WALL_DIST so the side toggle reaches the field Toggle,  */
/*         and tune TOGGLE_FLIP_DEGREES until one flip lands on your colour.  */
/*      5. Tune one routine at a time, shortest first.                        */
/*                                                                            */
/*    Keep the VEXcode Devices panel EMPTY and do not add "brain Brain;".     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

using namespace vex;

competition Competition;
controller Controller1 = controller(primary);

// 44W tank drive: two 11W motors per side.
// Right side is reversed so positive power drives both sides forward.
motor FrontLeft  = motor(PORT3,  ratio18_1, false);
motor BackLeft   = motor(PORT11, ratio18_1, false);
motor FrontRight = motor(PORT8,  ratio18_1, true);
motor BackRight  = motor(PORT20, ratio18_1, true);
motor_group LeftDrive  = motor_group(FrontLeft, BackLeft);
motor_group RightDrive = motor_group(FrontRight, BackRight);

inertial Inertial = inertial(PORT16);

// 22W cascade lift (two 11W motors). If they fight each other, flip one true/false.
motor LiftMotorA = motor(PORT1,  ratio18_1, false);
motor LiftMotorB = motor(PORT10, ratio18_1, true);
motor_group Lift = motor_group(LiftMotorA, LiftMotorB);

digital_out Claw = digital_out(Brain.ThreeWirePort.A);

// Side toggle mechanisms: one 5.5W motor on each side of the robot.
// (5.5W motors always run at 200 RPM, so ratio18_1 is correct for them.)
// If one spins the wrong way, flip its true/false.
motor ToggleLeft  = motor(PORT2, ratio18_1, false);
motor ToggleRight = motor(PORT9, ratio18_1, true);

/*---------------------------------------------------------------------------*/
/*                         NUMBERS TO TUNE                                   */
/*---------------------------------------------------------------------------*/

// ---- Driver ----
const int DEADBAND = 5;

// Drive, lift (driver) and side toggles are driven by voltage: 100% = 12 V.
// Voltage is open loop, so the auton minimum powers below are higher than
// they would be with percent velocity.
const double VOLTS_PER_PERCENT = 12.0 / 100.0;

// ---- Side toggles ----
const int    TOGGLE_SPEED = 50;             // percent. Lower = slower toggles
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
// Flip either of these to -1 if the robot drives or turns the wrong way.
const double DRIVE_SIGN  =  1;
const double TURN_SIGN   =  1;

// ---- Robot shape (inches / degrees) ----
// Angles are measured clockwise from the FRONT of the robot, looking down.
const double CLAW_ANGLE        = 0;     // the claw is on the front
const double TOGGLE_MECH_ANGLE = AUTON_TOGGLE_ON_RIGHT ? 90 : 270;  // side put against the wall
const double CLAW_REACH        = 12;    // robot centre -> middle of the claw
const double START_WALL_DIST   = 9;     // robot centre -> wall at the start (back on the wall)
// Robot centre -> wall while flipping. Must be at least 14 so an 18" robot
// can turn there without its corners hitting the wall; the side toggle has
// to reach the field Toggle from this far out (about 5" past the robot's side).
const double TOGGLE_WALL_DIST  = 14;
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
const double TURN_SPEED = 75;           // percent, cap for turns on the spot
const double MOVE_KP = 15.0;            // percent per inch of error
const double MOVE_KD = 0.05;
const double TURN_KP = 3.5;             // percent per degree of error
const double TURN_KD = 0.05;
const double MIN_MOVE = 20;             // smallest power that still moves (voltage)
const double MIN_TURN = 15;             // raise if turns stall just short of the angle
const double DRIVE_STEER_KP = 1.0;      // heading correction while driving
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
bool imuReady = false;          // set at the end of pre_auton, after calibrating

bool clawClosed = false;

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
// forward and turn (clockwise) are -100..100 percent.
void tankDrive(double forward, double turn) {
  forward = DRIVE_SIGN * forward;
  turn    = TURN_SIGN  * turn;

  double left  = forward + turn;
  double right = forward - turn;

  // Scale both sides down together if either is asked for more than 100%.
  double maxPower = std::fmax(std::fabs(left), std::fabs(right));
  if (maxPower > 100) {
    left  = left  / maxPower * 100;
    right = right / maxPower * 100;
  }

  LeftDrive.spin(fwd, left * VOLTS_PER_PERCENT, volt);
  RightDrive.spin(fwd, right * VOLTS_PER_PERCENT, volt);
}

// Speed (percent) a passThrough driveTo was still rolling at when it handed
// over, so the next move carries on from there instead of from a standstill.
double carriedSpeed = 0;
bool   carriedBackwards = false;

void stopDrive() {
  LeftDrive.stop(brake);
  RightDrive.stop(brake);
  carriedSpeed = 0;
}

/*---------------------------------------------------------------------------*/
/*      Odometry: where the robot is, from the inertial + wheel encoders     */
/*---------------------------------------------------------------------------*/
double robotX = 0, robotY = 0;      // inches, field coordinates
double headingOffset = 0;
double prevLeft = 0, prevRight = 0, prevHeading = 0;

// Field heading in degrees, clockwise, not wrapped.
double robotHeading() {
  return Inertial.rotation(degrees) + headingOffset;
}

double leftDeg()  { return (FrontLeft.position(degrees)  + BackLeft.position(degrees))  / 2.0; }
double rightDeg() { return (FrontRight.position(degrees) + BackRight.position(degrees)) / 2.0; }

void setPose(double x, double y, double heading) {
  robotX = x;
  robotY = y;
  headingOffset = heading - Inertial.rotation(degrees);
  prevHeading = heading;
  prevLeft = leftDeg();
  prevRight = rightDeg();
}

// Distance comes from the drive encoders, direction from the inertial.
void updateOdometry() {
  double l = leftDeg();
  double r = rightDeg();
  double dL = l - prevLeft, dR = r - prevRight;
  prevLeft = l; prevRight = r;

  double inchesPerDegree = M_PI * WHEEL_DIAMETER / 360.0 * DRIVE_GEAR_RATIO;
  double ahead = DRIVE_SIGN * (dL + dR) / 2.0 * inchesPerDegree * FORWARD_SCALE;

  double heading = robotHeading();
  double mid = (heading + prevHeading) / 2.0 * M_PI / 180.0;
  prevHeading = heading;

  robotX += ahead * std::sin(mid);
  robotY += ahead * std::cos(mid);
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

void checkAbort() {
  if (testRun && Controller1.ButtonDown.pressing()) {
    autonDeadline = 0;
  }
}

// Turn on the spot to a field heading.
void turnTo(double th, double maxSpeed, int timeoutMs) {
  if (timeLeft() <= 0) return;
  int end = nowMs() + timeoutMs;
  double prevErr = wrap180(th - robotHeading());
  int settled = 0;
  while (nowMs() < end && timeLeft() > 0) {
    updateOdometry();
    checkAbort();
    double err = wrap180(th - robotHeading());
    if (std::fabs(err) < HEAD_TOL) {
      settled += 10;
      if (settled >= SETTLE_MS) break;
    } else {
      settled = 0;
    }
    double turn = TURN_KP * err + TURN_KD * (err - prevErr) / 0.010;
    prevErr = err;
    if (turn > maxSpeed) turn = maxSpeed;
    if (turn < -maxSpeed) turn = -maxSpeed;
    if (std::fabs(err) > HEAD_TOL && std::fabs(turn) < MIN_TURN) {
      turn = (err > 0) ? MIN_TURN : -MIN_TURN;
    }
    tankDrive(0, turn);
    wait(10, msec);
  }
  stopDrive();
}

// Drive to a field point: turn to face it, then drive with heading
// correction. Goes backwards if the point is behind the robot, so backing
// away from a goal doesn't need a U-turn.
// passThrough = don't stop at the end, just get close and carry on.
// timeoutMs is for the driving part; the turn first gets its own time.
void driveTo(double tx, double ty, double maxSpeed, int timeoutMs, bool passThrough) {
  if (timeLeft() <= 0) return;
  double posTol = passThrough ? 3.0 : POS_TOL;

  double ex = tx - robotX, ey = ty - robotY;
  if (std::sqrt(ex * ex + ey * ey) < posTol) return;
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
  double prevAlong = 1e9;
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

    double speed = MOVE_KP * along;
    if (prevAlong < 1e8) speed += MOVE_KD * (along - prevAlong) / 0.010;
    prevAlong = along;
    if (passThrough && speed < PASS_SPEED) speed = PASS_SPEED;   // keep rolling
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed > lastSpeed + SLEW) speed = lastSpeed + SLEW;
    if (speed < MIN_MOVE) speed = MIN_MOVE;
    lastSpeed = speed;

    // Steer toward the point, but not in the last few inches where the
    // angle to it swings about.
    double turn = (dist > 6) ? DRIVE_STEER_KP * err : 0;
    if (turn > 40) turn = 40;
    if (turn < -40) turn = -40;

    tankDrive(backwards ? -speed : speed, turn);
    wait(10, msec);
  }
  if (passThrough) {
    carriedSpeed = lastSpeed;
    carriedBackwards = backwards;
  } else {
    stopDrive();
  }
}

// Drive to a field point, then (unless passThrough) turn to a heading.
void moveField(double tx, double ty, double th, double maxSpeed, int timeoutMs, bool passThrough) {
  if (timeLeft() <= 0) return;
  driveTo(tx, ty, maxSpeed, timeoutMs, passThrough);
  if (!passThrough) {
    turnTo(th, TURN_SPEED, 1500);
  }
}

// Go to a routine point with a chosen side of the robot (mechAngle) facing
// the routine direction faceDir.
void goTo(double cx, double cy, double faceDir, double mechAngle, double speed, int timeoutMs, bool passThrough) {
  moveField(fieldX(cx, cy), fieldY(cx, cy), fieldDir(faceDir) - mechAngle, speed, timeoutMs, passThrough);
}

// Drive to a routine point and stop there, without turning afterwards.
void driveToPoint(double cx, double cy, double speed, int timeoutMs) {
  driveTo(fieldX(cx, cy), fieldY(cx, cy), speed, timeoutMs, false);
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
  side.spin(reverseIt ? reverse : fwd, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
  while (std::fabs(side.position(degrees) - startPos) < TOGGLE_FLIP_DEGREES &&
         nowMs() - start < TOGGLE_AUTON_TIMEOUT_MS && timeLeft() > 0) {
    waitMs(10);
  }
  side.stop(hold);
}

// Flip the Toggle of the current quadrant: stand TOGGLE_WALL_DIST off the
// wall with the toggle side facing it, flip, then drive away (a Toggle only
// counts when no robot touches it).
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
  standOff(tx, ty, fromDir, CLAW_REACH + BACKOFF, CLAW_ANGLE, FAST, 2500, false);
  liftTo(height, true);
  standOff(tx, ty, fromDir, CLAW_REACH, CLAW_ANGLE, SLOW, 1500, false);
  autonClaw(true);
  waitMs(GRAB_MS);
  liftTo(height + H_LIFT_OFF, true);
  // Back off the full BACKOFF and stop, so the next turn has room.
  standOff(tx, ty, fromDir, CLAW_REACH + BACKOFF, CLAW_ANGLE, SLOW, 1000, false);
}

// Put what the claw is holding onto a goal (or onto the stack on it).
void placeOn(double gx, double gy, double fromDir, double height) {
  if (timeLeft() <= 0) return;
  liftTo(height + H_LIFT_OFF, false);
  standOff(gx, gy, fromDir, CLAW_REACH + BACKOFF, CLAW_ANGLE, FAST, 2500, false);
  liftTo(height + H_LIFT_OFF, true);
  standOff(gx, gy, fromDir, CLAW_REACH, CLAW_ANGLE, SLOW, 1500, false);
  liftTo(height, true);
  autonClaw(false);
  waitMs(RELEASE_MS);
  // Back off the full BACKOFF and stop, so the next turn has room.
  standOff(gx, gy, fromDir, CLAW_REACH + BACKOFF, CLAW_ANGLE, SLOW, 1000, false);
  liftTo(H_CARRY, false);
}

void finishAuton() {
  stopDrive();
  stopToggles();
  // Let a lift move that is still going (e.g. down to H_CARRY after the
  // last placeOn) finish, instead of freezing the lift where it is.
  int end = nowMs() + 1000;
  while (!Lift.isDone() && nowMs() < end && !(testRun && Controller1.ButtonDown.pressing())) {
    wait(10, msec);
  }
  Lift.stop(hold);
}

/*---------------------------------------------------------------------------*/
/*              Working in the corner by a wall stack                        */
/*---------------------------------------------------------------------------*/
// The wall stack and the goal beside it are only 21" apart, so the robot
// must not turn on the spot right next to them.
// Grabbing: line up further out on the same line the claw goes in on, so it
// turns there (about 15" from the goal) and then drives straight in and
// straight back out.
// Placing: there is no room on that line (it runs into the wall), so the
// robot turns at an open "stage" spot first.
// allianceSide = the stack / goal pair on the alliance-goal side of the Toggle.

// Robot centre -> wall stack while lining up to grab it.
const double CORNER_LINEUP = CLAW_REACH + BACKOFF + 6;

void cornerGrab(bool allianceSide, double height) {
  if (timeLeft() <= 0) return;
  double dir = allianceSide ? DIR_WALL_A : DIR_WALL_N;
  double sx = allianceSide ? WALL_A_X : WALL_N_X;
  double sy = allianceSide ? WALL_A_Y : WALL_N_Y;
  standOff(sx, sy, dir, CORNER_LINEUP, CLAW_ANGLE, FAST, 2500, false);
  grabAt(sx, sy, dir, height);
  // Back straight out along the same line before anything turns.
  standOff(sx, sy, dir, CORNER_LINEUP, CLAW_ANGLE, FAST, 1500, false);
}

void cornerPlace(bool allianceSide, double height) {
  if (timeLeft() <= 0) return;
  double dir = allianceSide ? DIR_GOAL_A_WALL : DIR_GOAL_N_WALL;
  goTo(STAGE_X, allianceSide ? STAGE_A_Y : STAGE_N_Y, dir + 180, CLAW_ANGLE, FAST, 2500, false);
  placeOn(allianceSide ? GOAL_A_X : GOAL_N_X, allianceSide ? GOAL_A_Y : GOAL_N_Y, dir, height);
  driveToPoint(STAGE_X, allianceSide ? STAGE_A_Y : STAGE_N_Y, FAST, 1500);
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
    if (!innerStackTaken) {
      // The straight line to (-10, -40) runs through the inner stack, so go
      // round it while it is still there.
      goTo(-8, -9, 180, TOGGLE_MECH_ANGLE, FAST, 2500, true);
    }
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
  // Waypoint first so the path stays clear of the inner field stack.
  driveTo(-36, -6, FAST, 2500, true);
  driveTo(-13, -11, FAST, 3000, false);
}

/*---------------------------------------------------------------------------*/
/*                              Test routine                                 */
/*---------------------------------------------------------------------------*/
// Forward 24", turn right, forward 24", then back to the start facing forward.
void testRoutine() {
  moveField(0, 24, 0, 60, 4000, false);
  waitMs(500);
  turnTo(90, TURN_SPEED, 2000);
  waitMs(500);
  moveField(24, 24, 90, 60, 4000, false);
  waitMs(500);
  moveField(0, 0, 0, 60, 6000, false);
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous                                   */
/*---------------------------------------------------------------------------*/
void runSelectedRoutine() {
  int which = routine;
  int budget = (which == R_SKILLS) ? 60000 : 15000;
  if (which == R_TEST) budget = 30000;
  autonDeadline = nowMs() + budget - 250;     // stop a touch early

  // If the robot was only just switched on, the inertial may still be
  // calibrating. Wait for it, or pre_auton would reset the pose mid-run.
  while (!imuReady) {
    wait(10, msec);
  }

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

  // Start: back against the wall under the Toggle, facing into the field.
  setPose(fieldX(TOGGLE_X + START_WALL_DIST, TOGGLE_Y),
          fieldY(TOGGLE_X + START_WALL_DIST, TOGGLE_Y),
          fieldDir(90));

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
  // Locked while an autonomous routine or an X+Y test-run is running.
  // (testRun only counts in driver control: if a test-run was cut off by
  // plugging into the field, the selector must still work while disabled.)
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
  }
  selectorDirty = true;
}

// Live readout under the selector, for tuning and troubleshooting.
void showStatus() {
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.setFillColor(color::black);
  Brain.Screen.printAt(10, 150, true, "X %6.1f  Y %6.1f  Heading %6.1f     ", robotX, robotY, robotHeading());
  Brain.Screen.printAt(10, 175, true, "Lift %5.0f deg   Claw %s     ", LiftMotorA.position(degrees), clawClosed ? "closed" : "open  ");
  Brain.Screen.printAt(10, 200, true, "Side toggles: left(2) %s  right(9) %s   ",
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
  imuReady = true;
}

/*---------------------------------------------------------------------------*/
/*                              User Control                                 */
/*---------------------------------------------------------------------------*/
void showDriveMode() {
  Controller1.Screen.clearLine(1);
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("93601V tank drive");
}

void usercontrol(void) {
  // Make sure nothing from autonomous or a cut-off test-run is still running.
  autonDeadline = 0;
  testRun = false;
  selectorDirty = true;
  stopToggles();
  showDriveMode();

  // Without a field or switch, driver control starts while the inertial is
  // still calibrating. Moving the robot then would spoil the calibration.
  while (!imuReady) {
    wait(20, msec);
  }

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
      // Wait for X and Y to be let go, so one press runs the routine once
      // and an abort with Down stays aborted.
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

    // Lift: hold L1 to go up, L2 to go down, holds position when released.
    if (Controller1.ButtonL1.pressing()) {
      Lift.spin(fwd, LIFT_SPEED * VOLTS_PER_PERCENT, volt);
    } else if (Controller1.ButtonL2.pressing()) {
      Lift.spin(reverse, LIFT_SPEED * VOLTS_PER_PERCENT, volt);
    } else {
      Lift.stop(hold);
    }

    // Side toggles: hold R2 to spin both. Hold the Up arrow to spin both
    // the other way.
    if (Controller1.ButtonR2.pressing()) {
      ToggleLeft.spin(fwd, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
      ToggleRight.spin(fwd, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
    } else if (Controller1.ButtonUp.pressing()) {
      ToggleLeft.spin(reverse, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
      ToggleRight.spin(reverse, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
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
