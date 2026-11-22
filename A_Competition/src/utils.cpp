#include "utils.h"
#include "config.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.1415926535897932386
#endif

static constexpr double INCH_TO_CM = 2.54;
static constexpr double CIRCUMFERENCE_CM =
    M_PI * Config::trackingWheelDiameterInch * INCH_TO_CM;  // ~15.959 cm for 2"

double degree_to_cm(double degree) {
    return (degree / 360.0) * CIRCUMFERENCE_CM;
}

double cm_to_degree(double centimeters) {
    return (centimeters / CIRCUMFERENCE_CM) * 360.0;
}

double clampP(double x, double min, double max){
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

double curveStick(double percent, double strength) {
    strength = clampP(strength, 0, 1);
    double x = clampP(percent, -100, 100) / 100.0;
    // Blend between a straight line and a cubic.
    double y = (1 - strength) * x + strength * x * x * x;
    return y * 100.0;
}

double slewToward(double current, double target,
                  double accelStep, double decelStep) {
    if (target == current) return target;

    // Opposite signs: first come back to 0 at the decel rate, then speed up
    // the other way on later calls.
    if (current * target < 0) {
        if (decelStep <= 0) return 0;
        if (std::fabs(current) <= decelStep) return 0;
        return current > 0 ? current - decelStep : current + decelStep;
    }

    bool speedingUp = std::fabs(target) > std::fabs(current);
    double step = speedingUp ? accelStep : decelStep;
    if (step <= 0) return target;  // no limit in this direction

    double delta = target - current;
    if (delta > step) return current + step;
    if (delta < -step) return current - step;
    return target;
}
