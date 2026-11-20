#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
using namespace vex;

extern brain Brain;
extern controller Controller1;

// VEXcode devices
extern drivetrain Drivetrain;
extern motor LeftMotorA;
extern motor LeftMotorB;
extern motor RightMotorA;
extern motor RightMotorB;
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;

extern inertial TestInertial;
extern rotation Rotation2;
// extern vision Vision1;  // vision sensor not installed yet


extern motor LiftMotorA;
extern motor LiftMotorB;
extern motor_group Lift;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );

#endif // ROBOT_CONFIG_H
