#ifndef VISION_H
#define VISION_H

#include <stdint.h>
#include "pid.h"

// Call repeatedly. Target distance is in Rotation2 degrees from the reset origin.
// Forward only; returns true when the distance threshold is reached or crossed.
// Stops and returns false if the sensor or target is unavailable.
bool setupVisionDrive(pid& controller, double targetDistanceDegrees,
                      double uncertaintyCm = 1.0);

#endif // VISION_H
