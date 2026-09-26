#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
using namespace vex;

extern brain Brain;
extern controller Controller1;

// VEXcode devices
extern drivetrain Drivetrain;
extern motor LeftDriveSmart;
extern motor RightDriveSmart;

extern inertial TestInertial;
extern rotation Rotation2;


extern motor ClawMotor;
extern motor ArmMotor;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );

#endif // ROBOT_CONFIG_H