#include "vex.h"

using namespace vex;

// Change device ports here when the wiring changes.
namespace RobotPorts {
const int liftMotorA = PORT8; //red
const int liftMotorB = PORT2;
const int leftMotorA = PORT17;
const int leftMotorB = PORT9;
const int leftMotorC = POR21;

const int rightMotorA = PORT12;
const int rightMotorB = PORT15;
const int rightMotorC = PORT4;

const int carousle
const int inertial = PORT6;
const int rotation = PORT19;
}

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller1 = controller();

// VEXcode device constructors
// Lift: 2 motors in one group. The two motors face each other,
// so one is reversed. If they fight each other, flip ONE true/false; if the
// whole lift goes the wrong way, flip BOTH.
motor LiftMotorA = motor(RobotPorts::liftMotorA, ratio36_1, false); // Lift Left
motor LiftMotorB = motor(RobotPorts::liftMotorB, ratio36_1, false); // Lift Right
motor_group Lift = motor_group(LiftMotorA, LiftMotorB);

// Drive: 2 motors per side. Flip the true/false if a motor spins the wrong way.
motor LeftMotorA = motor(RobotPorts::leftMotorA, ratio6_1, false);
motor LeftMotorB = motor(RobotPorts::leftMotorB, ratio6_1, false);
motor RightMotorA = motor(RobotPorts::rightMotorA, ratio6_1, true);
motor RightMotorB = motor(RobotPorts::rightMotorB, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(LeftMotorA, LeftMotorB);
motor_group RightDriveSmart = motor_group(RightMotorA, RightMotorB);
drivetrain Drivetrain = drivetrain
(LeftDriveSmart, //left motor
  RightDriveSmart, //right motor
  319.19, // Wheel travel
  295, //track width 
  40, // wheelbase
   mm, 1);

inertial TestInertial = inertial(RobotPorts::inertial);
rotation Rotation2 = rotation(RobotPorts::rotation, false);

// Vision sensor not installed yet.
// Re-enable on a free port later.
// BLUE_TARGET is signature slot 1 from the Vision Utility export.
// signature BLUE_TARGET(
//     1, -1885, -1609, -1747, 1285, 1643, 1464, 2.5, 0
// );
// vision Vision1(PORT1, 50, BLUE_TARGET);

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
  
  //setup the speed of motors
  Lift.setStopping(hold);  // keep the lift from dropping when released
  Lift.setVelocity(90, percent);

  //add a delay to allow the inertial sensor to calibrate
  wait(30,msec);

  vex::thread screenThread(screenRefresh);
}

void screenRefresh() {
    while (true) {
        controller_screen();
        brain_screen();
        wait(200, msec);
    }
}
