#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
using namespace vex;

extern brain Brain;
extern controller Controller1;

// VEXcode devices
extern motor LeftMotorA;
extern motor LeftMotorB;
extern motor LeftMotorC;
extern motor RightMotorA;
extern motor RightMotorB;
extern motor RightMotorC;
extern motor_group LeftDrive;
extern motor_group RightDrive;

extern inertial InertialSensor;             // heading
extern rotation RotationSensor;   // distance (rotation sensor on the tracking wheel)
// extern vision Vision1;  // vision sensor not installed yet


extern motor LiftMotorA;
extern motor LiftMotorB;
extern motor_group Lift;

extern motor IntakeMotorA;
extern motor IntakeMotorB;
extern motor_group Intake;

extern motor ClawMotorA;
extern motor ClawMotorB;
extern motor_group Claw;

// Piston on three-wire port A
extern digital_out Piston;

// Cup orienter
extern optical OpticalSensor;
extern motor CascadeMotor;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );

#endif // ROBOT_CONFIG_H
