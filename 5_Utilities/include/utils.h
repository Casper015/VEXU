#ifndef UTILS_H
#define UTILS_H

// 3.25 inch wheel constants
constexpr double WHEEL_DIAMETER_INCH = 3.25;
constexpr double INCH_TO_METER = 0.0254;
constexpr double WHEEL_DIAMETER_M = WHEEL_DIAMETER_INCH * INCH_TO_METER; 

double degree_to(const double degree, const bool inch_or_m);
double degree_to_m(double degree);
double degree_to_inch(double degree);

double distance_to_degree(const double distance, const bool inch_or_m);
double m_to_degree(double meters);
double inch_to_degree(double inches);

#endif // UTILS_H