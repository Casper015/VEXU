#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller1 = controller();

// VEXcode device constructors
motor ClawMotor = motor(PORT1, ratio6_1, false);
motor ArmMotor = motor(PORT2, ratio6_1, false);

motor LeftDrive1 = motor(PORT8, ratio6_1, false);
motor LeftDrive2 = motor(PORT9, ratio6_1, false);
motor_group LeftDriveSmart = motor_group(LeftDrive1, LeftDrive2);

motor RightDrive1 = motor(PORT11, ratio6_1, true);
motor RightDrive2 = motor(PORT18, ratio6_1, true);
motor_group RightDriveSmart = motor_group(RightDrive1, RightDrive2);

drivetrain Drivetrain = drivetrain
(LeftDriveSmart, //left motor
  RightDriveSmart, //right motor
  319.19, // Wheel travel
  295, //track width 
  40, // wheelbase
   mm, 1);

inertial TestInertial = inertial(PORT14);
rotation Rotation2 = rotation(PORT10, false);

// VEXcode generated functions

void vexcodeInit();
void screenRefresh();

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit() {
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

  vex::thread screenThread(screenRefresh);
}

void screenRefresh() {
    while (true) {
        controller_screen();
        wait(200, msec);
    }
}