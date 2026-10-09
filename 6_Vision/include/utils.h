#ifndef UTILS_H
#define UTILS_H

// inch_or_cm: true = inches, false = centimeters
double degree_to(const double degree, const bool inch_or_cm);
double degree_to_cm(double degree);
double degree_to_inch(double degree);

double distance_to_degree(const double distance, const bool inch_or_cm);
double cm_to_degree(double centimeters);
double inch_to_degree(double inches);

double clampP(double x, double min, double max);

#endif // UTILS_H
