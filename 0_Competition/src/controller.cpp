#include "vex.h"


using namespace vex;

// Refresh one line, then return so the turn code can continue.
void controller_screen() {
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("I:%3.1f degrees R:%3.1f degrees",
      TestInertial.rotation(degrees), Rotation2.position(degrees));
}

void controller_L1_Pressed(){
  ArmMotor.spin(forward);
  while (Controller1.ButtonL1.pressing()) {
    wait(5, msec);
  }
  ArmMotor.stop();
}

void controller_L2_Pressed(){
  ArmMotor.spin(reverse);
  while (Controller1.ButtonL2.pressing()) {
    wait(5, msec);
  }
  ArmMotor.stop();
}

// Hold R1 to raise the lift; release to stop (motors hold position).
void controller_R1_Pressed(){
  Lift.spin(forward);
  while (Controller1.ButtonR1.pressing()) {
    wait(5, msec);
  }
  Lift.stop();
}

// Hold R2 to lower the lift; release to stop (motors hold position).
void controller_R2_Pressed(){
  Lift.spin(reverse);
  while (Controller1.ButtonR2.pressing()) {
    wait(5, msec);
  }
  Lift.stop();
}

// Claw not installed yet. R1/R2 now belong to the lift, so pick other
// buttons when the claw is re-enabled.
/*
void controller_claw_close(){   // was on R1
  ClawMotor.spin(reverse);
  while (Controller1.ButtonR1.pressing()) {
    wait(5, msec);
  }
  ClawMotor.stop();
}

void controller_claw_open(){    // was on R2
  ClawMotor.spin(forward);
  while (Controller1.ButtonR2.pressing()) {
    wait(5, msec);
  }
  ClawMotor.stop();
}
*/

// Hold X to run the intake forward (in); release to stop.
void controller_X_Pressed(){
  Intake.spin(forward);
  while (Controller1.ButtonX.pressing()) {
    wait(5, msec);
  }
  Intake.stop();
}

// Hold Y to run the intake in reverse (out); release to stop.
void controller_Y_Pressed(){
  Intake.spin(reverse);
  while (Controller1.ButtonY.pressing()) {
    wait(5, msec);
  }
  Intake.stop();
}

// Hold B to run the carousel; release to stop.
void controller_B_Pressed(){
  Carousel.spin(forward);
  while (Controller1.ButtonB.pressing()) {
    wait(5, msec);
  }
  Carousel.stop();
}
