#include "vex.h"


using namespace vex;

// Controller screen, line 1: I = inertial heading (deg), R = tracking wheel
// (deg). Kept short because the controller only shows about 19 characters.
void controller_screen() {
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("I:%.1f R:%.1f    ",
      InertialSensor.rotation(degrees), RotationSensor.position(degrees));
}

// Every motor in the program, in the order shown on the PORT CHECK screen
// (left column first, then right column).
struct MotorEntry {
  const char* name;
  motor* device;
};

static MotorEntry allMotors[] = {
  {"WhlLA", &LeftMotorA},   {"WhlRA", &RightMotorA},
  {"WhlLB", &LeftMotorB},   {"WhlRB", &RightMotorB},
  {"WhlLC", &LeftMotorC},   {"WhlRC", &RightMotorC},
  {"LiftL", &LiftMotorA},   {"LiftR", &LiftMotorB},
  {"IntkA", &IntakeMotorA}, {"IntkB", &IntakeMotorB},
  {"ClawA", &ClawMotorA},   {"ClawB", &ClawMotorB},
  {"Cascd", &CascadeMotor},
};
static const int motorCount = sizeof(allMotors) / sizeof(allMotors[0]);

// True if another motor in the program uses the same port as this one.
static bool portUsedTwice(int which) {
  for (int i = 0; i < motorCount; i++) {
    if (i != which &&
        allMotors[i].device->index() == allMotors[which].device->index()) {
      return true;
    }
  }
  return false;
}

// One cell per motor: name, the port the code uses for it, its state and its
// real speed. State is OK (motor plugged in), -- (nothing on that port) or
// DUP (two motors in the code share this port, which is always a mistake).
static void motor_cell(int which) {
  motor& m = *allMotors[which].device;
  const char* state = portUsedTwice(which) ? "DUP" : (m.installed() ? "OK" : "--");

  Brain.Screen.setCursor(2 + which / 2, (which % 2 == 0) ? 1 : 25);
  Brain.Screen.print("%-5s p%2d %-3s %4d%%   ", allMotors[which].name,
                     (int)m.index() + 1, state,
                     (int)m.velocity(velocityUnits::pct));
}

// PORT CHECK screen: work one stick or button at a time and watch which cell
// shows a speed. The cell that moves tells you which port is really driven.
void brain_screen() {
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("PORT CHECK  LeftY %4d LeftX %4d RightY %4d   ",
                     (int)Controller1.Axis3.position(),
                     (int)Controller1.Axis4.position(),
                     (int)Controller1.Axis2.position());
  for (int i = 0; i < motorCount; i++) {
    motor_cell(i);
  }
}
