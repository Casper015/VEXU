#ifndef UTILS_H
#define UTILS_H

// 3.25 inch wheel constants
constexpr double WHEEL_DIAMETER_INCH = 3.25;
constexpr double INCH_TO_METER = 0.0254;
constexpr double WHEEL_DIAMETER_M = WHEEL_DIAMETER_INCH * INCH_TO_METER; // 0.08255 m

// 1. 角度转距离 (degree -> distance)
// inch_or_m: true 为 inch, false 为 meter
double degree_to(const double degree, const bool inch_or_m);
double degree_to_m(double degree);
double degree_to_inch(double degree);

// 2. 距离转角度 (distance -> degree)
// inch_or_m: true 为 inch, false 为 meter
double distance_to_degree(const double distance, const bool inch_or_m);
double m_to_degree(double meters);
double inch_to_degree(double inches);

#endif // UTILS_H