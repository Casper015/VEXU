#include "utils.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.1415926535897932386
#endif

static constexpr double CIRCUMFERENCE_INCH = M_PI * WHEEL_DIAMETER_INCH; // ~10.210176 inch
static constexpr double CIRCUMFERENCE_M    = M_PI * WHEEL_DIAMETER_M;    // ~0.2593385 m

double degree_to_inch(double degree) {
    return (degree / 360.0) * CIRCUMFERENCE_INCH;
}

double degree_to_m(double degree) {
    return (degree / 360.0) * CIRCUMFERENCE_M;
}

double degree_to(const double degree, const bool inch_or_m) {
    return inch_or_m ? degree_to_inch(degree) : degree_to_m(degree);
}

double inch_to_degree(double inches) {
    return (inches / CIRCUMFERENCE_INCH) * 360.0;
}

double m_to_degree(double meters) {
    return (meters / CIRCUMFERENCE_M) * 360.0;
}

double distance_to_degree(const double distance, const bool inch_or_m) {
    return inch_or_m ? inch_to_degree(distance) : m_to_degree(distance);
}
