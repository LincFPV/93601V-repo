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

// 22W cascade lift: two 11W motors working together.
// If one motor fights the others, flip its true/false.
motor LiftMotorA = motor(PORT1, ratio18_1, false);
motor LiftMotorB = motor(PORT2, ratio18_1, true);
motor_group Lift = motor_group(LiftMotorA, LiftMotorB);

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
  // Forward/back and turning are inverted; strafing is not.
  drive = -drive;
  turn = -turn;

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
