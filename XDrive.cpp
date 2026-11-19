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
/*      Lift (2 x 11W) .... 1, 2      <- change these to match your robot     */
/*      Side toggles ...... left 3, right 4 (5.5W motors)                     */
/*      Claw piston ....... 3-wire port A on the brain                        */
/*                                                                            */
/*    Controls:                                                               */
/*      Left stick Y  = forward / backward                                    */
/*      Left stick X  = strafe left / right                                   */
/*      Right stick X = turn                                                  */
/*      L1 / L2       = lift up / down                                        */
/*      R1            = open / close claw                                     */
/*      R2 (hold)       = spin both side toggles                              */
/*      Up arrow (hold) = spin both side toggles the other way                */
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

// 22W cascade lift: two 11W motors working together.
// If one motor fights the others, flip its true/false.
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

bool fieldCentric = true;
bool clawClosed = false;

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

void usercontrol(void) {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(fieldCentric ? "Field-centric" : "Robot-centric");

  while (true) {
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
