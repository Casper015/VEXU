#ifndef JOYSTICK_H
#define JOYSTICK_H

// Joystick shaping: turns a raw stick reading (-100..100) into a command that
// is comfortable to drive with. This file is pure math and touches no robot
// devices. The numbers that tune it are in config.h ("how the drive feels").
//
// The usual order is:  deadband -> curve -> top speed -> acceleration limit
// -> left/right mix.

// Deadband. Readings inside +-deadband give 0, so a stick that does not rest
// exactly at centre cannot make the robot creep. Outside the deadband the
// output starts from 0 and rises smoothly to 100 at full stick, so there is
// no sudden jump when the stick leaves the deadband.
double stickDeadband(double raw, double deadband);

// Response curve. percent is -100..100, strength is 0..1.
// strength 0 returns percent unchanged; higher values make the output smaller
// near the centre (finer control) while +-100 still gives +-100.
double curveStick(double percent, double strength);

// Deadband, then curve, then scale so full stick gives maxPercent.
double shapeStick(double raw, double deadband, double curve, double maxPercent);

// Acceleration limit. Moves current toward target by at most accelStep when
// the command is growing (away from 0) and at most decelStep when it is
// shrinking (toward 0). A step of 0 means "no limit" for that direction.
// Call once per loop with step = percentPerSecond * loopSeconds.
double slewToward(double current, double target,
                  double accelStep, double decelStep);

// Arcade mix: forward and turn commands -> left and right side commands.
// If either side would go past +-100, both are scaled down together, so the
// robot keeps the same turning arc instead of losing its turn at full speed.
void mixArcade(double forwardCmd, double turnCmd, double& left, double& right);

#endif // JOYSTICK_H
