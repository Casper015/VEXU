#include "vex.h"


using namespace vex;

// Refresh one line, then return so the turn code can continue.
void controller_screen() {
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("I:%3.1f degrees R:%3.1f degrees",
      TestInertial.rotation(degrees), Rotation2.position(degrees));
}

// One Brain-screen line per motor: its name in the code, the port the code
// uses for it, whether a motor is plugged into that port, and its real speed.
static void motor_row(int row, const char* name, motor& m) {
  Brain.Screen.setCursor(row, 1);
  Brain.Screen.print("%-8s port %2d  %-4s %4d%%   ", name,
                     (int)m.index() + 1, m.installed() ? "OK" : "NONE",
                     (int)m.velocity(velocityUnits::pct));
}

// PORT CHECK screen: push one stick at a time and watch which rows show a
// speed. The row that moves tells you which port is really being driven.
void brain_screen() {
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("PORT CHECK  LeftY %4d LeftX %4d RightY %4d   ",
                     (int)Controller1.Axis3.position(),
                     (int)Controller1.Axis4.position(),
                     (int)Controller1.Axis2.position());
  motor_row(3, "Lift L", LiftMotorA);
  motor_row(4, "Lift R", LiftMotorB);
  motor_row(5, "Wheel LA", LeftMotorA);
  motor_row(6, "Wheel LB", LeftMotorB);
  motor_row(7, "Wheel RA", RightMotorA);
  motor_row(8, "Wheel RB", RightMotorB);
}
