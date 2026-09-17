#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller1 = controller();

// VEXcode device constructors
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
  // nothing to initialize
}