#include "vex.h"


using namespace vex;

// Refresh one line, then return so the turn code can continue.
void controller_screen() {
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("I:%3.1f degrees R:%3.1f degrees",
      InertialSensor.rotation(degrees), RotationSensor.position(degrees));
  // What the program is sending to the clamp solenoid on three-wire A.
  // If this flips ON/OFF with R1/R2 but the clamp does not move, the problem
  // is the wiring or the air, not the code.
  // Also the lift height in degrees, used to set liftUpDegrees in main.cpp.
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Clmp:%-3s Lift:%5.0f ", Piston.value() ? "ON" : "OFF",
                           Lift.position(degrees));
}

// One Brain-screen line per motor: its name in the code, the port the code
// uses for it, whether a motor is plugged into that port, its real speed and
// its current draw. High current with 0% speed = the motor is stalled
// (blocked, or fighting the other motor in its group).
static void motor_row(int row, const char* name, motor& m) {
  Brain.Screen.setCursor(row, 1);
  Brain.Screen.print("%-8s port %2d  %-4s %4d%% %4.1fA   ", name,
                     (int)m.index() + 1, m.installed() ? "OK" : "NONE",
                     (int)m.velocity(velocityUnits::pct),
                     m.current(currentUnits::amp));
}

// PORT CHECK screen: push one stick at a time and watch which rows show a
// speed. The row that moves tells you which port is really being driven.
void brain_screen() {
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("PORT CHECK  LeftY %4d LeftX %4d RightY %4d   ",
                     (int)Controller1.Axis3.position(),
                     (int)Controller1.Axis4.position(),
                     (int)Controller1.Axis2.position());  // RightY = claw
  motor_row(3, "Lift L", LiftMotorA);
  motor_row(4, "Lift R", LiftMotorB);
  motor_row(5, "Wheel LA", LeftMotorA);
  motor_row(6, "Wheel LB", LeftMotorB);
  motor_row(7, "Wheel LC", LeftMotorC);
  motor_row(8, "Wheel RA", RightMotorA);
  motor_row(9, "Wheel RB", RightMotorB);
  motor_row(10, "Wheel RC", RightMotorC);
  motor_row(11, "Claw A", ClawMotorA);
  motor_row(12, "Claw B", ClawMotorB);
}
