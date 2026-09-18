
#include "vex.h"


using namespace vex;

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

void controller_R1_Pressed(){
  ClawMotor.spin(reverse);
  while (Controller1.ButtonR1.pressing()) {
    wait(5, msec);
  }
  ClawMotor.stop();
}

void controller_R2_Pressed(){
  ClawMotor.spin(forward);
  while (Controller1.ButtonR2.pressing()) {
    wait(5, msec);
  }
  ClawMotor.stop();
}