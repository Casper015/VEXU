#include "cmath"
#include <stdint.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"
#include "pid.h"

using namespace vex;

double calculatePID(pid& controller, double target, double measurement);
void resetPID(pid& controller);

static double calculateI(pid& controller, double error, uint32_t dtMS);
static inline double clamp(double x, double min, double max);

// need to implement the I term for the PID controller
double calculatePID(pid& controller, double target, 
    double measurement){

    //setup P base on the error multiplied by the proportional gain
    const double error = target - measurement;
    const double pOutput = controller.kp * error;

    //all the ts is base on the ms(uint32_t), 
    //put this in the design of all the others code.
    uint32_t nowMs = Brain.Timer.time();
    uint32_t dtMS = nowMs - controller.previousTime;


    //calculate D based on the change in measurement over time
    double dOutput = 0;
    if(controller.firstRun){
    }else if(dtMS > 0){
        dOutput = -controller.kd * 
        (measurement - controller.previousMeasurement)/dtMS;
    }

    controller.previousTime = nowMs;
    controller.previousError = error;
    controller.previousMeasurement = measurement;
    controller.firstRun = false;

    calculateI(controller, error, dtMS);

    const double output = pOutput + dOutput;
    return clamp(output, -controller.maxOutput, controller.maxOutput);

}

void resetPID(pid& controller){
    controller.previousError = 0;
    controller.previousMeasurement = 0;
    controller.integral = 0;
    controller.firstRun = true;
}

//do not finish yet, maybe implement I term later
static double calculateI(pid& controller, double error, uint32_t dtMS){
       /*
    Implement the I term here
    */
    return 0;
}

// Utility function to clamp a value between a minimum and maximum.
static inline double clamp(double x, double min, double max){
    if (x < min) return min;
    if (x > max) return max;
    return x;
}
