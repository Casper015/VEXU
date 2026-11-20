#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
using namespace vex;

// Change this to the Smart Port the Optical Sensor is plugged into.
#define OPTICAL_PORT PORT1

extern brain Brain;
extern optical Optical1;

void vexcodeInit(void);

#endif // ROBOT_CONFIG_H
