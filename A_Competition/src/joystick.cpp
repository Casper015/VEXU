#include "joystick.h"
#include "utils.h"
#include <cmath>

double stickDeadband(double raw, double deadband) {
    raw = clampP(raw, -100, 100);
    deadband = clampP(deadband, 0, 99);

    double size = std::fabs(raw);
    if (size <= deadband) return 0;

    // Stretch the part outside the deadband back over the full 0..100 range.
    double scaled = (size - deadband) / (100.0 - deadband) * 100.0;
    return raw > 0 ? scaled : -scaled;
}

double curveStick(double percent, double strength) {
    strength = clampP(strength, 0, 1);
    double x = clampP(percent, -100, 100) / 100.0;
    // Blend between a straight line and a cubic.
    double y = (1 - strength) * x + strength * x * x * x;
    return y * 100.0;
}

double shapeStick(double raw, double deadband, double curve, double maxPercent) {
    double shaped = curveStick(stickDeadband(raw, deadband), curve);
    return shaped * clampP(maxPercent, 0, 100) / 100.0;
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

void mixArcade(double forwardCmd, double turnCmd, double& left, double& right) {
    left = forwardCmd + turnCmd;
    right = forwardCmd - turnCmd;

    double biggest = std::fmax(std::fabs(left), std::fabs(right));
    if (biggest > 100.0) {
        left = left / biggest * 100.0;
        right = right / biggest * 100.0;
    }
}
