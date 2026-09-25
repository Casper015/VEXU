#include "vex.h"

using namespace vex;

bool setupTurn(pid& controller, double targetAngle, double uncertainty = 2.0);
bool setupDrive(pid& controller, double targetDistance, double uncertainty = 2.0);

int main() {
  vexcodeInit();
  while (TestInertial.isCalibrating()) {
    wait(20, msec);
  }

  double target_inertial = 90;
  double target_rotation2 = 720;

  // First test: use P control for the 90-degree right turn.
  pid turnPID{};
  turnPID.kp = 0.65;
  turnPID.ki = 5;
  turnPID.kd = 0;
  turnPID.maxOutput = 50;

  pid drivePID{};
  drivePID.kp = 0.12;
  drivePID.kd = 0.5;
  drivePID.maxOutput = 60;

  int step = 1;

  TestInertial.setRotation(0, degrees);

  while(step <= 4){
    switch (step) {
    case 1:
      // 1. 右转 90 度
      if (setupTurn(turnPID, target_inertial, 2)) {
        resetPID(turnPID);
        TestInertial.setRotation(0, degrees);
        step++;
      }
      break;
    case 2:
      // 2. 左转 90 度 (-90)
      if (setupTurn(turnPID, -target_inertial, 2)) {
        resetPID(drivePID);
        Rotation2.resetPosition();
        step++;
      }
      break;
    case 3:
      // 3. 前进 720 度
      if (setupDrive(drivePID, target_rotation2, 10)) {
        resetPID(drivePID);
        Rotation2.resetPosition();
        step++;
      }
      break;
    case 4:
      // 4. 后退 720 度 (-720)
      if (setupDrive(drivePID, -target_rotation2, 10)) {
        step++;
      }
      break;
    }
    wait(20, msec);
  }

  Drivetrain.stop(brake);
  controller_screen();
}

bool setupTurn(pid& controller, double targetAngle, 
  double uncertainty) {
    
    double currentAngle = TestInertial.rotation(degrees);
    double error = targetAngle - currentAngle;

    if (fabs(error) <= uncertainty) {
      Drivetrain.stop(brake); 
      return true;
    }

    double speed = calculatePID(controller, targetAngle, currentAngle);
    if (speed >= 0) {
      Drivetrain.turn(right, speed, velocityUnits::pct);
    } else {
      Drivetrain.turn(left, -speed, velocityUnits::pct);
    }
    return false;
}

bool setupDrive(pid& controller, double targetDistance, 
  double uncertainty) {
    
    double currentDistance = Rotation2.position(degrees);
    double error = targetDistance - currentDistance;

    if (fabs(error) <= uncertainty) {
      Drivetrain.stop(brake); 
      return true;
    }

    double speed = calculatePID(controller, targetDistance, currentDistance);
    if (speed >= 0) {
      Drivetrain.drive(forward, speed, velocityUnits::pct);
    } else {
      Drivetrain.drive(reverse, -speed, velocityUnits::pct);
    }
    return false;
}