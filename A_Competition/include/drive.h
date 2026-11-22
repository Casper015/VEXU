#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

// Turn to an ABSOLUTE heading in degrees (0 = heading at the last
// resetHeading(), positive = right). Blocks until settled or timeout.
// timeoutMs = 0 picks a timeout from the size of the turn.
void turnPID(double angleDeg, uint32_t timeoutMs = 0);

// Drive a RELATIVE distance in cm (negative = backward), holding the current
// target heading. Blocks until settled or timeout.
// timeoutMs = 0 picks a timeout from the distance.
void drivePID(double distanceCm, uint32_t timeoutMs = 0);

// Brake both sides of the drive.
void stopDrive();

// Make the direction the robot faces right now heading 0.
void resetHeading();

// Auto-test cancel. While allowed, pressing Down on the controller stops the
// current turnPID()/drivePID() and makes every later one return at once.
// Calling this (true or false) also clears a previous cancel.
void allowDriveAbort(bool allowed);

#endif // DRIVE_H
