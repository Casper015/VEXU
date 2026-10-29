#include "vex.h"
#include "vision.h"
#include <cmath>

using namespace vex;

namespace {
const double visionCenterX = 158.0;
const double visionKp = 0.20;
const double visionDeadband = 4.0;
const double maxVisionCorrection = 12.0;

// Change to -1.0 if the robot steers away from the target.
const double steeringSign = 1.0;
}

bool setupVisionDrive(pid& controller, double targetDistanceDegrees,
                      double uncertaintyCm) {
  const double currentDistance = Rotation2.position(degrees);
  const double remaining = targetDistanceDegrees - currentDistance;

  // Forward-only arrival: do not reverse after crossing the distance threshold.
  if (remaining <= cm_to_degree(uncertaintyCm)) {
    Drivetrain.stop(brake);
    return true;
  }

  if (!Vision1.installed()) {
    Drivetrain.stop(brake);
    resetPID(controller);
    return false;
  }

  Vision1.takeSnapshot(BLUE_TARGET, 1);
  if (Vision1.objectCount <= 0 || !Vision1.largestObject.exists) {
    Drivetrain.stop(brake);
    resetPID(controller);
    return false;
  }

  // Distance PID sets forward speed; vision sets the steering correction.
  double speed = calculatePID(controller, targetDistanceDegrees, currentDistance);
  speed = clampP(speed, 0.0, controller.maxOutput);

  double visionError = Vision1.largestObject.centerX - visionCenterX;
  if (std::fabs(visionError) <= visionDeadband) {
    visionError = 0.0;
  }

  // Keep both wheels moving forward as distance PID slows the robot down.
  const double correctionLimit = std::fmin(maxVisionCorrection, speed * 0.7);
  const double correction = clampP(steeringSign * visionKp * visionError,
                                  -correctionLimit, correctionLimit);

  // Shared compensation ratios are declared in robot-config.h.
  double leftSpeed = (speed + correction) * leftRatio;
  double rightSpeed = (speed - correction) * rightRatio;

  // Scale together so output saturation preserves the left/right speed ratio.
  const double peak = std::fmax(std::fabs(leftSpeed), std::fabs(rightSpeed));
  if (peak > controller.maxOutput) {
    const double scale = controller.maxOutput / peak;
    leftSpeed *= scale;
    rightSpeed *= scale;
  }

  LeftDriveSmart.spin(forward, leftSpeed, velocityUnits::pct);
  RightDriveSmart.spin(forward, rightSpeed, velocityUnits::pct);
  return false;
}
