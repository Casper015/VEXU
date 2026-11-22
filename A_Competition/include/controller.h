#ifndef CONTROLLER_H
#define CONTROLLER_H

// Ouput the Rotation and Inertial sensor values to the controller screen
void controller_screen();

// Show every motor's port, plugged-in state and live speed on the Brain screen
void brain_screen();

// Counts every pass of the driver-control loop in main.cpp. The Brain screen
// shows it, so a number that keeps climbing proves driver control is running.
extern volatile unsigned long driverLoopCount;

#endif
