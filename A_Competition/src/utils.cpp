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
