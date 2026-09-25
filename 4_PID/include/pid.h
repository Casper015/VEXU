#ifndef PID_H
#define PID_H

// PID controller structure and functions
/*
Everytime want to setup a new PID controller
pid newPID; set up the 
newPID.kp = 0;
newPID.ki = 0;
newPID.maxOutput = 0;

how to use this PID controller
double speed = calculatePID( 
newPID, 
yourTarget,
yourMeasurement）
*/
struct pid
{
    double kp = 0;
    double ki = 0;
    double kd = 0;

    double maxOutput = 100;
    double integralRange = 0;
    double maxIntegralOutput = 0;

    uint32_t previousTime = 0;
    double previousError = 0;
    double previousMeasurement = 0;
    double integral = 0;
    bool firstRun = true;
}; 

// reset all the pid to the initial state
void resetPID(pid& controller);

// main function to calculate the PID output
double calculatePID(pid& controller, double target, 
    double measurement);
#endif // PID_H