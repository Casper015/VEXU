#ifndef UTILS_H
#define UTILS_H

// Tracking-wheel conversions: rotation-sensor degrees <-> distance in cm.
// The wheel diameter is Config::trackingWheelDiameterInch.
double degree_to_cm(double degree);
double cm_to_degree(double centimeters);

// Limit x to the range [min, max].
double clampP(double x, double min, double max);

// Joystick response curve. percent is -100..100, strength is 0..1.
// strength 0 returns percent unchanged; higher values make the output smaller
// near the centre while +-100 still gives +-100.
double curveStick(double percent, double strength);

// Acceleration limit. Moves current toward target by at most accelStep when
// the command is growing (away from 0) and at most decelStep when it is
// shrinking (toward 0). A step of 0 means "no limit" for that direction.
// Call once per loop with step = percentPerSecond * loopSeconds.
double slewToward(double current, double target,
                  double accelStep, double decelStep);

#endif // UTILS_H
