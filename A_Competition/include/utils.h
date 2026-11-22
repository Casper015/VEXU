#ifndef UTILS_H
#define UTILS_H

// Tracking-wheel conversions: rotation-sensor degrees <-> distance in cm.
// The wheel diameter is Config::trackingWheelDiameterInch.
double degree_to_cm(double degree);
double cm_to_degree(double centimeters);

// Limit x to the range [min, max].
double clampP(double x, double min, double max);

#endif // UTILS_H
