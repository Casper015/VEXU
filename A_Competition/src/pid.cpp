#include <cmath>

#include "vex.h"

using namespace vex;

double calculatePID(pid& ctrl, double target, double measurement){
    if (!std::isfinite(target) || !std::isfinite(measurement) ||
        !std::isfinite(ctrl.maxOutput) ||
        ctrl.maxOutput <= 0 || ctrl.maxOutput > 100) {
        resetPID(ctrl);
        return 0;
    }

    // P: error multiplied by the proportional gain
    const double error = target - measurement;
    const double pOutput = ctrl.kp * error;

    // all times are in ms (uint32_t)
    uint32_t nowMs = Brain.Timer.time();
    uint32_t dtMS = nowMs - ctrl.previousTime;

    // D: based on the change in measurement over time (skipped on first run)
    double dOutput = 0;
    if (!ctrl.firstRun && dtMS > 0) {
        dOutput = -ctrl.kd *
        (measurement - ctrl.previousMeasurement)/dtMS;
    }

    ctrl.previousTime = nowMs;
    ctrl.previousMeasurement = measurement;
    ctrl.firstRun = false;

    const double output = pOutput + dOutput;
    if(!std::isfinite(output)){
        resetPID(ctrl);
        return 0;
    }
    return clampP(output, -ctrl.maxOutput, ctrl.maxOutput);
}

void resetPID(pid& ctrl){
    ctrl.previousMeasurement = 0;
    ctrl.firstRun = true;

    ctrl.settleStartMs = 0;
    ctrl.settling = false;
}

bool hasSettled(pid& ctrl, double error,
    double uncertainty, uint32_t settleTimeMs) {
    uint32_t nowMs = Brain.Timer.time();

    if (std::fabs(error) > uncertainty) {
        ctrl.settling = false;
        return false;
    }

    if (!ctrl.settling) {
        ctrl.settleStartMs = nowMs;
        ctrl.settling = true;
    }

    return (nowMs - ctrl.settleStartMs) >= settleTimeMs;
}
