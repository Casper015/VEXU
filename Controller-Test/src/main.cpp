/* ---------------------------------------------------------------------------- */
/*                                                                              */                 
/*    Project:        Clawbot Controller with Events                            */
/*    Module:         main.cpp                                                  */
/*    Author:         VEX                                                       */
/*    Created:        Fri Aug 05 2022                                           */
/*    Description:    This example will use Controller button events to         */
/*                    control the V5 Clawbot arm and claw                       */
/*                                                                              */                 
/*    Configuration:  V5 Clawbot (Individual Motors)                            */
/*                    Controller                                                */
/*                    Claw Motor in Port 3                                      */
/*                    Arm Motor in Port 8                                       */
/*                    Left Motor in Port 1                                      */
/*                    Right Motor in Port 10                                    */
/*                                                                              */                 
/* ---------------------------------------------------------------------------- */

#include "vex.h"

using namespace vex;

// Robot configuration code.

// Brain should be defined by default
brain Brain;

motor ClawMotor = motor(PORT3, ratio18_1, false);
motor ArmMotor = motor(PORT8, ratio18_1, false);
motor LeftMotor = motor(PORT1, ratio18_1, false);
motor RightMotor = motor(PORT10, ratio18_1, true);

controller Controller1 = controller(primary);

// Begin project code

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

int main() {

  // Create Controller callback events - 15 msec delay to ensure events get registered
  Controller1.ButtonL1.pressed(controller_L1_Pressed);
  Controller1.ButtonL2.pressed(controller_L2_Pressed);
  Controller1.ButtonR1.pressed(controller_R1_Pressed);
  Controller1.ButtonR2.pressed(controller_R2_Pressed);
  wait(15,msec);

  // Configure Arm and Claw motor hold settings and velocity
  ArmMotor.setStopping(hold);
  ClawMotor.setStopping(hold);
  ArmMotor.setVelocity(60, percent);
  ClawMotor.setVelocity(30, percent);

  // Main Controller loop to set motors to controller axis postiions
  while(true){
    LeftMotor.setVelocity(Controller1.Axis3.position(), percent);
    RightMotor.setVelocity(Controller1.Axis2.position(), percent);
    LeftMotor.spin(forward);
    RightMotor.spin(forward);
    wait(5, msec);
  }
}
