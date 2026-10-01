#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller1 = controller();

// VEXcode device constructors
motor ClawMotor = motor(PORT8, ratio18_1, false);
motor ArmMotor = motor(PORT13, ratio18_1, false);

motor LeftDriveSmart = motor(PORT10, ratio18_1, false);
motor RightDriveSmart = motor(PORT11, ratio18_1, true);
drivetrain Drivetrain = drivetrain
(LeftDriveSmart, //left motor
  RightDriveSmart, //right motor
  319.19, // Wheel travel
  295, //track width 
  40, // wheelbase
   mm, 1);

inertial TestInertial = inertial(PORT17);
rotation Rotation2 = rotation(PORT18, false);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  //reset devices and calibrate sensors
  Rotation2.resetPosition();
  TestInertial.resetRotation();
  TestInertial.calibrate();
  wait(30,msec);

  //setup the controller screen
  controller_screen();
  
  //setup the button
  Controller1.ButtonL1.pressed(controller_L1_Pressed);
  Controller1.ButtonL2.pressed(controller_L2_Pressed);
  Controller1.ButtonR1.pressed(controller_R1_Pressed);
  Controller1.ButtonR2.pressed(controller_R2_Pressed);

  //setup the speed of motors
  ArmMotor.setStopping(hold);
  ClawMotor.setStopping(hold);
  ArmMotor.setVelocity(60, percent);
  ClawMotor.setVelocity(30, percent);

  //add a delay to allow the inertial sensor to calibrate
  wait(30,msec);
}