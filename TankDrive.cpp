/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Team 93601V - Tank drive, driver control only (4 x 11W = 44W)           */
/*    For matches use OverrideComp.v5cpp, which also has the autons.          */
/*                                                                            */
/*    Ports:                                                                  */
/*      Radio (antenna) ... 11  (no code needed, just plug it in)             */
/*      Left drive ........ 10 (front), 9 (back)                              */
/*      Right drive ....... 20 (front), 19 (back)                             */
/*      Lift (2 x 11W) .... 1, 2                                              */
/*      Side toggles ...... left 3, right 4 (5.5W motors)                     */
/*      Claw piston ....... 3-wire port A on the brain                        */
/*                                                                            */
/*    Controls:                                                               */
/*      Left stick up/down  = forward / backward                              */
/*      Right stick L/R     = turn                                            */
/*      L1 / L2             = lift up / down                                  */
/*      R1                  = open / close claw                               */
/*      R2 (hold)           = spin both side toggles                          */
/*      Up arrow (hold)     = spin both side toggles the other way            */
/*                                                                            */
/*    If the robot drives backwards when you push forward, set DRIVE_SIGN     */
/*    to -1. If it turns the wrong way, set TURN_SIGN to -1.                  */
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
motor FrontLeft  = motor(PORT10, ratio18_1, false);
motor BackLeft   = motor(PORT9,  ratio18_1, false);
motor FrontRight = motor(PORT20, ratio18_1, true);
motor BackRight  = motor(PORT19, ratio18_1, true);
motor_group LeftDrive  = motor_group(FrontLeft, BackLeft);
motor_group RightDrive = motor_group(FrontRight, BackRight);

// 22W cascade lift: two 11W motors working together.
// If the motors fight each other, flip one true/false.
motor LiftMotorA = motor(PORT1, ratio18_1, false);
motor LiftMotorB = motor(PORT2, ratio18_1, true);
motor_group Lift = motor_group(LiftMotorA, LiftMotorB);

// Pneumatic claw: solenoid plugged into 3-wire port A.
digital_out Claw = digital_out(Brain.ThreeWirePort.A);

// Side toggle mechanisms: one 5.5W motor on each side of the robot.
// (5.5W motors always run at 200 RPM, so ratio18_1 is correct for them.)
// If one spins the wrong way, flip its true/false.
motor ToggleLeft  = motor(PORT3, ratio18_1, false);
motor ToggleRight = motor(PORT4, ratio18_1, true);

// Joystick values below this are treated as 0 to stop drift.
const int DEADBAND = 5;

const int TOGGLE_SPEED = 100;         // percent (5.5W motors are weak)

// Flip either of these to -1 if the robot drives or turns the wrong way.
const double DRIVE_SIGN = 1;
const double TURN_SIGN  = 1;

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

  LeftDrive.spin(fwd, left, percent);
  RightDrive.spin(fwd, right, percent);
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

void toggleClaw() {
  setClaw(!clawClosed);
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
  // The real autons are in OverrideComp.v5cpp.
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

  while (true) {
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
      Lift.spin(fwd, 100, percent);
    } else if (Controller1.ButtonL2.pressing()) {
      Lift.spin(reverse, 100, percent);
    } else {
      Lift.stop(hold);
    }

    // Side toggles: hold R2 to spin both. Hold the Up arrow to spin both
    // the other way. They stop as soon as you let go.
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

    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Registered here, not in usercontrol, so each press only fires once
  // even when the field or switch restarts driver control.
  Controller1.ButtonR1.pressed(toggleClaw);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
