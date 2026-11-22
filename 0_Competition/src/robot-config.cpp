#include "vex.h"

using namespace vex;
//arm green motor

// Change device ports here when the wiring changes.
namespace RobotPorts {
const int liftMotorA = PORT2; //red
const int liftMotorB = PORT8;

const int leftMotorA = PORT17; 
const int leftMotorB = PORT9;
const int leftMotorC = PORT21;

const int rightMotorA = PORT12;
const int rightMotorB = PORT15;
const int rightMotorC = PORT4;

// Everything below is a PLACEHOLDER port, set these once the wiring is decided.
const int cascadeMotor = PORT10;

const int intakeMotorA = PORT6;
const int intakeMotorB = PORT16;

const int clawMotorA = PORT20;
const int clawMotorB = PORT5;

// The piston is on three-wire port A. Three-wire ports are not numbers, so
// its port is set where Piston is defined, just below the Brain.

const int optical = PORT14;  // PLACEHOLDER: was 16, which is intakeMotorB
const int vision = PORT19;
const int inertial = PORT13;
const int rotation = PORT11;

}

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller1 = controller();

// Piston: three-wire port A on the side of the Brain.
// Piston.set(true) extends it, Piston.set(false) retracts it.
digital_out Piston = digital_out(Brain.ThreeWirePort.A);

// VEXcode device constructors
// Lift: 2 motors in one group. The two motors face each other,
// so one is reversed. If they fight each other, flip ONE true/false; if the
// whole lift goes the wrong way, flip BOTH.
motor LiftMotorA = motor(RobotPorts::liftMotorA, ratio36_1, false); // Lift Left
motor LiftMotorB = motor(RobotPorts::liftMotorB, ratio36_1, true); // Lift Right
motor_group Lift = motor_group(LiftMotorA, LiftMotorB);

// Drive: 3 motors per side. All three on a side must turn the wheels the same
// way; flip the true/false of any motor that fights the other two.
motor LeftMotorA = motor(RobotPorts::leftMotorA, ratio6_1, true);
motor LeftMotorB = motor(RobotPorts::leftMotorB, ratio6_1, true);
motor LeftMotorC = motor(RobotPorts::leftMotorC, ratio6_1, false);

motor RightMotorA = motor(RobotPorts::rightMotorA, ratio6_1, false);
motor RightMotorB = motor(RobotPorts::rightMotorB, ratio6_1, false);
motor RightMotorC = motor(RobotPorts::rightMotorC, ratio6_1, true);

motor_group LeftDrive = motor_group(LeftMotorA, LeftMotorB, LeftMotorC);
motor_group RightDrive = motor_group(RightMotorA, RightMotorB, RightMotorC);

// Intake: 2 motors in one group. Set the gear cartridge to match the real
// motors, and flip true/false so both pull the same way.
motor IntakeMotorA = motor(RobotPorts::intakeMotorA, ratio18_1, false);
motor IntakeMotorB = motor(RobotPorts::intakeMotorB, ratio18_1, true);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);

// Claw: 2 motors in one group. Same notes as the intake.
motor ClawMotorA = motor(RobotPorts::clawMotorA, ratio6_1, false);
motor ClawMotorB = motor(RobotPorts::clawMotorB, ratio6_1, true);
motor_group Claw = motor_group(ClawMotorA, ClawMotorB);

// Inertial sensor: heading. Rotation sensor on the tracking wheel: distance.
// RotationSensor must count UP when the robot drives forward; if it counts
// down, change false to true.
inertial InertialSensor = inertial(RobotPorts::inertial);
rotation RotationSensor = rotation(RobotPorts::rotation, false);

// Cascade: the rotating part in the middle of the intake (1 motor). The cup
// orienter uses it to turn a cup round after the Optical Sensor has looked at
// one end of the cup. Flip true/false if it turns the wrong way.
optical OpticalSensor = optical(RobotPorts::optical);
motor CascadeMotor = motor(RobotPorts::cascadeMotor, ratio18_1, false);

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
  RotationSensor.resetPosition();
  InertialSensor.resetRotation();
  InertialSensor.calibrate();
  wait(30,msec);  // give the calibrating flag time to come up before it is polled

  // keep the lift from dropping when released
  Lift.setStopping(hold);
  // lift height 0 = where the lift is now, so start with the lift at the bottom
  Lift.resetPosition();
  // keep the claw gripping when the button is released
  Claw.setStopping(hold);
  // keep the cascade locked at the end of each 90 degree step
  CascadeMotor.setStopping(hold);

  vex::thread screenThread(screenRefresh);
}

void screenRefresh() {
    while (true) {
        controller_screen();
        brain_screen();
        wait(200, msec);
    }
}
