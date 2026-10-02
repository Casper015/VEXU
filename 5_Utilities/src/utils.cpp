#include <iostream>

// d = 3.25 inches

double degree_to (const double degree, const bool inch_or_m) {
    if (inch_or_m) {
        return (degree / 360) * 3.14 * 3.25;
    } else {
        return (degree / 360) * 3.14 * 3.25 * 0.0254;
    }
}
