#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// Every tunable number lives here.
// Device ports are at the top of src/robot-config.cpp, next to the devices.
namespace Config {

// ---- Driver control ----
// Joystick values below this are treated as 0 (stops drift when released).
constexpr int joystickDeadband = 5;
// Full right-stick deflection runs the lift at this speed (percent).
constexpr double liftMaxPercent = 90;
// Intake speed while R1 (in) or R2 (out) is held (percent).
constexpr double intakeSpeedPercent = 100;
// Claw speed while L1 (close) or L2 (open) is held (percent).
constexpr double clawSpeedPercent = 50;
// Cascade (the rotating part in the middle) speed while X (forward) or
// B (reverse) is held (percent).
constexpr double cascadeSpeedPercent = 50;
// How often the driver loop runs (ms).
constexpr int driverLoopMs = 20;

// ---- Driver control: how the drive feels ----
// Stick curve, 0 to 1. 0 = straight line (stick 50% -> speed 50%).
// Higher = gentler around the centre for fine moves, full stick is still
// full speed. At 1, stick 50% gives only 12.5% speed.
constexpr double forwardCurve = 0.3;
constexpr double turnCurve = 0.5;
// Top speed at full stick (percent). Lower turnMaxPercent for calmer turns.
constexpr double forwardMaxPercent = 100;
constexpr double turnMaxPercent = 100;
// Acceleration limit: how fast the command may rise, in percent per second.
// 400 means 0 -> 100% takes 0.25 s. 0 = no limit (instant).
constexpr double forwardAccelPerSec = 400;
constexpr double turnAccelPerSec = 0;
// Deceleration limit: how fast the command may fall back toward 0.
// 0 = no limit, the robot stops as soon as the stick is released.
constexpr double forwardDecelPerSec = 0;
constexpr double turnDecelPerSec = 0;
// To get the old behaviour back: both curves 0, all four limits 0.

// ---- Testing aids (check these before a real competition) ----
// Up button in driver control runs autonomous() once; Down cancels it.
// Set to false before a real competition.
constexpr bool enableAutoTestButton = true;
// Show the PORT CHECK table on the Brain screen.
constexpr bool showPortCheck = true;

// Show the live Optical Sensor readings and the cup decision on the Brain.
constexpr bool showCupDebug = true;

// ---- Cup orienter (Optical Sensor + cascade motor) ----
// Master switch. false = the A button and orientCup() do nothing.
constexpr bool enableCupOrienter = true;

// Sensor settings.
constexpr double cupLedPercent = 100;    // LED power, 0-100
constexpr double cupIntegrationMs = 20;  // lower = faster but noisier

// Which reading decides opaque vs clear:
//   0 = saturation (use with a strongly coloured background behind the cup)
//   1 = brightness (use with a matte black background)
//   2 = proximity  (infrared reflection, no background change needed)
constexpr int cupDetectBy = 0;
// Thresholds for that reading. Read the real numbers off the Brain screen
// (showCupDebug) with each end of a cup in front of the sensor, then set:
//   cupOpaqueThreshold = a value just inside the opaque-end readings
//   cupClearThreshold  = a value just inside the clear-end readings
// Readings between the two are "not sure" and are ignored. Either one may
// be the larger number. While both are equal (0 and 0) the orienter is
// treated as not calibrated and never turns the table.
constexpr double cupOpaqueThreshold = 0;
constexpr double cupClearThreshold = 0;
// This many readings in a row must agree before a decision counts.
constexpr int cupAgreeingReads = 5;
// Give up deciding after this long.
constexpr uint32_t cupDetectTimeoutMs = 500;

// Which end should face the sensor when the cup is the right way round.
constexpr bool cupWantOpaqueAtSensor = true;
// Cascade move when the cup is the wrong way round.
constexpr double cupFlipDegrees = 180;       // degrees of the TABLE
constexpr double cupTableGearRatio = 1.0;    // motor degrees per table degree
constexpr double cupFlipSpeedPercent = 60;
constexpr uint32_t cupFlipTimeoutMs = 1000;

// ---- Tracking wheel (the wheel on the rotation sensor) ----
constexpr double trackingWheelDiameterInch = 2.0;

// ---- Autonomous: PID gains ----
// Taken from pid (1).h: kP = 0.98, kD = 0.67, turn limited to 15%, drive
// limited to 50%. That code computed D as (error change per 10 ms loop) and
// drive error in cm; calculatePID() uses change per ms and drive error in
// rotation-sensor degrees, so drive.cpp scales the gains to behave the same.
constexpr double srcKp = 0.98;
constexpr double srcKd = 0.67;
constexpr double srcLoopMs = 10;
constexpr double turnMaxOutput = 15;   // percent
constexpr double driveMaxOutput = 50;  // percent

// ---- Autonomous: driving straight ----
// Per-side compensation so the robot drives straight.
constexpr double leftRatio = 1.0;
constexpr double rightRatio = 1.0;
// Heading hold while driving straight.
constexpr double headingKp = 1;
constexpr double maxCorrection = 8.0;  // percent

// ---- Autonomous: when a move counts as finished ----
// Error must stay inside the tolerance for the settle time.
constexpr double turnToleranceDeg = 1;
constexpr uint32_t turnSettleMs = 50;
constexpr double driveToleranceCm = 1;
constexpr uint32_t driveSettleMs = 150;

// ---- Autonomous: timeouts (a move gives up after base + per-unit time) ----
constexpr uint32_t turnTimeoutBaseMs = 1000;
constexpr double turnTimeoutMsPerDeg = 25;
constexpr uint32_t driveTimeoutBaseMs = 1500;
constexpr double driveTimeoutMsPerCm = 20;

}  // namespace Config

#endif // CONFIG_H
