#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller1 = controller();

// VEXcode device constructors
motor ClawMotor = motor(PORT8, ratio18_1, false);

// Intake: 2 motors in one group. TODO: set the real ports / gear cartridge,
// and flip true/false so both motors pull game pieces in the same direction.
motor IntakeMotorA = motor(PORT1, ratio18_1, false);
motor IntakeMotorB = motor(PORT10, ratio18_1, true);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);

motor ArmMotor = motor(PORT13, ratio18_1, false);

// Drive: 2 motors per side. TODO: set LeftMotorB / RightMotorB to the real
// ports, and flip the true/false if that motor spins the wrong way.
motor LeftMotorA = motor(PORT2, ratio18_1, false);
motor LeftMotorB = motor(PORT3, ratio18_1, false);
motor RightMotorA = motor(PORT8, ratio18_1, true);
motor RightMotorB = motor(PORT9, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(LeftMotorA, LeftMotorB);
motor_group RightDriveSmart = motor_group(RightMotorA, RightMotorB);
drivetrain Drivetrain = drivetrain
(LeftDriveSmart, //left motorx 
  RightDriveSmart, //right motor
  319.19, // Wheel travel
  295, //track width 
  40, // wheelbase
   mm, 1);

inertial TestInertial = inertial(PORT6);
rotation Rotation2 = rotation(PORT20, false);

// BLUE_TARGET is signature slot 1 from the Vision Utility export.
signature BLUE_TARGET(
    1, -1885, -1609, -1747, 1285, 1643, 1464, 2.5, 0
);
vision Vision1(PORT1, 50, BLUE_TARGET);

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
  Controller1.ButtonA.pressed(controller_A_Pressed);

  //setup the speed of motors
  ArmMotor.setStopping(hold);
  ClawMotor.setStopping(hold);
  ArmMotor.setVelocity(60, percent);
  ClawMotor.setVelocity(30, percent);
  Intake.setVelocity(100, percent);

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