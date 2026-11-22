#ifndef PID_H
#define PID_H

#include <stdint.h>

// PD controller structure and functions.
/*
Set up a new controller:
  pid newPID;
  newPID.kp = 0;
  newPID.kd = 0;
  newPID.maxOutput = 100;

Use it once per loop:
  double speed = calculatePID(newPID, yourTarget, yourMeasurement);
*/
struct pid
{
    double kp = 0;
    double kd = 0;

    // Output is limited to +-maxOutput (must be > 0 and <= 100).
    double maxOutput = 100;

    // Internal state, managed by the functions below.
    uint32_t settleStartMs = 0;
    bool settling = false;

    uint32_t previousTime = 0;
    double previousMeasurement = 0;
    bool firstRun = true;
};

// reset the controller to its initial state (gains are kept)
void resetPID(pid& ctrl);

// main function to calculate the PID output
double calculatePID(pid& ctrl, double target, double measurement);

// check if the error has stayed within uncertainty for settleTimeMs
bool hasSettled(pid& ctrl, double error,
    double uncertainty, uint32_t settleTimeMs = 150);

#endif // PID_H
