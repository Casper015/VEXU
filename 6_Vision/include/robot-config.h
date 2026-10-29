#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
using namespace vex;

extern brain Brain;
extern controller Controller1;

// VEXcode devices
const double leftRatio = 0.95;
const double rightRatio = 1.0;

extern drivetrain Drivetrain;
extern motor LeftDriveSmart;
extern motor RightDriveSmart;

extern inertial TestInertial;
extern rotation Rotation2;

extern motor ClawMotor;
extern motor ArmMotor;

extern vex::vision::signature BLUE_TARGET;
extern vision Vision1;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );

#endif // ROBOT_CONFIG_H