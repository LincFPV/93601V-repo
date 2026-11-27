/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Team 93601V - Tank drive, driver control only (4 x 11W = 44W)           */
/*    For matches use TankComp.v5cpp, which also has the autons.              */
/*                                                                            */
/*    Ports:                                                                  */
/*      Left drive ........ 3 (front), 11 (back)                              */
/*      Right drive ....... 8 (front), 20 (back)                              */
/*      Lift (2 x 11W) .... 1, 10                                             */
/*      Side toggles ...... left 2, right 9 (5.5W motors)                     */
/*      Claw piston ....... 3-wire port A on the brain                        */
/*      Radio (antenna) ... any free port (no code needed)                    */
/*                                                                            */
/*    Drive, lift and side toggles run on voltage (12 V = full power) for     */
/*    more speed and punch than percent velocity.                             */
/*                                                                            */
/*    Controls:                                                               */
/*      Left stick up/down  = forward / backward                              */
/*      Right stick L/R     = turn                                            */
/*      R1 / R2             = lift up / down                                  */
/*      L1 / L2             = close / open claw                               */
/*      A (hold)            = spin both side toggles                          */
/*      Up arrow (hold)     = spin both side toggles the other way            */
/*      B                   = arcade / tank controls (not on a field)         */
/*                                                                            */
/*    The drivetrain is inverted in the code (DRIVE_SIGN and TURN_SIGN are    */
/*    -1). If a direction is wrong, change that one back to 1.                */
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

// 22W cascade lift: two 11W motors working together.
// If the motors fight each other, flip one true/false.
motor LiftMotorA = motor(PORT1,  ratio18_1, false);
motor LiftMotorB = motor(PORT10, ratio18_1, true);
motor_group Lift = motor_group(LiftMotorA, LiftMotorB);

// Pneumatic claw: solenoid plugged into 3-wire port A.
digital_out Claw = digital_out(Brain.ThreeWirePort.A);

// Side toggle mechanisms: one 5.5W motor on each side of the robot.
// (5.5W motors always run at 200 RPM, so ratio18_1 is correct for them.)
// If one spins the wrong way, flip its true/false.
motor ToggleLeft  = motor(PORT2, ratio18_1, false);
motor ToggleRight = motor(PORT9, ratio18_1, true);

// Joystick values below this are treated as 0 to stop drift.
const int DEADBAND = 5;

// Motors are driven by voltage: 100% = 12 V.
const double VOLTS_PER_PERCENT = 12.0 / 100.0;

const int TOGGLE_SPEED = 50;          // percent. Lower = slower toggles
const int LIFT_SPEED   = 100;         // percent

// The drivetrain is inverted: -1 flips forward/back (DRIVE_SIGN) and turning
// (TURN_SIGN). If one of them goes the wrong way, change it back to 1.
const double DRIVE_SIGN = -1;
const double TURN_SIGN  = -1;

bool clawClosed = false;

int applyDeadband(int value) {
  return (std::abs(value) < DEADBAND) ? 0 : value;
}

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

void stopDrive() {
  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

/*---------------------------------------------------------------------------*/
/*                               Claw                                        */
/*---------------------------------------------------------------------------*/
void setClaw(bool closed) {
  clawClosed = closed;
  Claw.set(closed);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous                                   */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  Lift.setStopping(hold);
  ToggleLeft.setStopping(hold);
  ToggleRight.setStopping(hold);
  setClaw(false);

  Brain.Screen.clearScreen();
  Brain.Screen.printAt(10, 30, true, "93601V tank drive ready");
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous                                   */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  // Placeholder: drive forward 1 second, then stop.
  // The real autons are in TankComp.v5cpp.
  tankDrive(50, 0);
  wait(1000, msec);
  stopDrive();
}

/*---------------------------------------------------------------------------*/
/*                              User Control                                 */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("93601V tank drive");

  bool tankMode = false;          // practice only: B switches to tank controls
  bool bWasPressed = false;
  bool shownTank = true;          // forces the first controller-screen update

  while (true) {
    // Practice only: B switches between arcade and tank controls. On a field
    // (field control or competition switch) it is always arcade.
    bool onField = Competition.isFieldControl() || Competition.isCompetitionSwitch();
    bool bPressed = Controller1.ButtonB.pressing();
    if (bPressed && !bWasPressed && !onField) tankMode = !tankMode;
    bWasPressed = bPressed;
    if (onField) tankMode = false;
    if (tankMode != shownTank) {
      shownTank = tankMode;
      Controller1.Screen.clearLine(3);
      Controller1.Screen.setCursor(3, 1);
      Controller1.Screen.print(tankMode ? "TANK controls (B)" : "ARCADE controls");
    }

    if (tankMode) {
      // Tank: left stick drives the left side, right stick the right side.
      double left  = applyDeadband(Controller1.Axis3.position());
      double right = applyDeadband(Controller1.Axis2.position());
      if (left == 0 && right == 0) {
        stopDrive();
      } else {
        tankDrive((left + right) / 2, (left - right) / 2);
      }
    } else {
      // Arcade: left stick up/down drives, right stick left/right turns.
      double forward = applyDeadband(Controller1.Axis3.position());
      double turn    = applyDeadband(Controller1.Axis1.position());
      if (forward == 0 && turn == 0) {
        stopDrive();
      } else {
        tankDrive(forward, turn);
      }
    }

    // Lift: hold R1 to go up, R2 to go down, holds position when released.
    if (Controller1.ButtonR1.pressing()) {
      Lift.spin(fwd, LIFT_SPEED * VOLTS_PER_PERCENT, volt);
    } else if (Controller1.ButtonR2.pressing()) {
      Lift.spin(reverse, LIFT_SPEED * VOLTS_PER_PERCENT, volt);
    } else {
      Lift.stop(hold);
    }

    // Claw: L1 closes it (grab), L2 opens it (let go).
    if (Controller1.ButtonL1.pressing()) {
      setClaw(true);
    } else if (Controller1.ButtonL2.pressing()) {
      setClaw(false);
    }

    // Side toggles: hold A to spin both. Hold the Up arrow to spin both
    // the other way. They stop as soon as you let go.
    if (Controller1.ButtonA.pressing()) {
      ToggleLeft.spin(fwd, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
      ToggleRight.spin(fwd, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
    } else if (Controller1.ButtonUp.pressing()) {
      ToggleLeft.spin(reverse, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
      ToggleRight.spin(reverse, TOGGLE_SPEED * VOLTS_PER_PERCENT, volt);
    } else {
      ToggleLeft.stop(hold);
      ToggleRight.stop(hold);
    }

    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
