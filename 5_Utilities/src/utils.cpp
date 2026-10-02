#include "utils.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.1415926535897932386
#endif

double WHEEL_DIAMETER_INCH = 2.0;
double INCH_TO_CM = 2.54;
double WHEEL_DIAMETER_CM = WHEEL_DIAMETER_INCH * INCH_TO_CM;

double CIRCUMFERENCE_INCH = M_PI * WHEEL_DIAMETER_INCH; // ~6.283185 inch
double CIRCUMFERENCE_CM    = M_PI * WHEEL_DIAMETER_CM;    // ~15.959291 cm

double degree_to_inch(double degree) {
    return (degree / 360.0) * CIRCUMFERENCE_INCH;
}

double degree_to_cm(double degree) {
    return (degree / 360.0) * CIRCUMFERENCE_CM;
}

double degree_to(const double degree, const bool inch_or_cm) {
    return inch_or_cm ? degree_to_inch(degree) : degree_to_cm(degree);
}

double inch_to_degree(double inches) {
    return (inches / CIRCUMFERENCE_INCH) * 360.0;
}

double cm_to_degree(double centimeters) {
    return (centimeters / CIRCUMFERENCE_CM) * 360.0;
}

double distance_to_degree(const double distance, const bool inch_or_cm) {
    return inch_or_cm ? inch_to_degree(distance) : cm_to_degree(distance);
}

double clampP(double x, double min, double max){
    if (x < min) return min;
    if (x > max) return max;
    return x;
}