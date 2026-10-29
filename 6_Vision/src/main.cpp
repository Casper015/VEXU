#include "vex.h"
#include "vision.h"

using namespace vex;

bool setupTurn(pid& controller, double targetAngle, double uncertainty = 2);
bool setupDrive(pid& controller, double targetDistance, double targetHeading, double uncertainty_cm = 2.0);

const double headingKp = 1;
const double maxCorrection = 8.0;

int main() {
  vexcodeInit();
  while (TestInertial.isCalibrating()) {
    wait(20, msec);
  }

  if (!Vision1.installed()) {
    Drivetrain.stop(brake);
    Brain.Screen.print("Vision disconnected");
    return 0;
  }

  pid drivePID{};
  drivePID.kp = 0.08;
  drivePID.kd = 5.0;
  drivePID.maxOutput = 35.0;

  resetPID(drivePID);
  Rotation2.resetPosition();

  // Drive forward 50 cm while keeping BLUE_TARGET horizontally centered.
  const double targetDistance = cm_to_degree(50.0);
  const double startMs = Brain.Timer.time();
  const double timeoutMs = 15000.0;
  bool reached = false;

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Tracking BLUE...");

  while (Brain.Timer.time() - startMs < timeoutMs) {
    if (setupVisionDrive(drivePID, targetDistance, 1.0)) {
      reached = true;
      break;
    }
    wait(20, msec);
  }

  Drivetrain.stop(brake);
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print(reached ? "Finished" : "Timed out");
  return 0;
}

bool setupTurn(pid& controller, double targetAngle, 
  double uncertainty) {
    
    double currentAngle = TestInertial.rotation(degrees);
    double error = targetAngle - currentAngle;
    
    if (hasSettled(controller, error, uncertainty, 50)) {
      Drivetrain.stop(brake); 
      return true;
    }

    double speed = calculatePID(controller, targetAngle, currentAngle);
    Drivetrain.turn(right, speed, velocityUnits::pct);
    
    return false;
}

bool setupDrive(pid& controller, double targetDistance, double targetHeading, 
  double uncertainty_cm) {
    
    double currentDistance = Rotation2.position(degrees);
    double uncertainty_degree = cm_to_degree(uncertainty_cm);
    double DistanceError = targetDistance - currentDistance; 

    if (hasSettled(controller, DistanceError, uncertainty_degree, 150)) {
      Drivetrain.stop(brake); 
      return true;
    }

    double speed = calculatePID(controller, targetDistance, currentDistance);

    double headingError = targetHeading - TestInertial.rotation(degrees);

    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;
    
    

    double correction = clampP(headingKp * headingError, -maxCorrection, maxCorrection);

    // Apply each side's compensation to the combined drive and heading command.
    double leftSpeed = clampP((speed + correction) * leftRatio, -controller.maxOutput, controller.maxOutput);
    double rightSpeed = clampP((speed - correction) * rightRatio, -controller.maxOutput, controller.maxOutput);

    LeftDriveSmart.spin(forward, leftSpeed, velocityUnits::pct);
    RightDriveSmart.spin(forward, rightSpeed, velocityUnits::pct);

    return false;
}

