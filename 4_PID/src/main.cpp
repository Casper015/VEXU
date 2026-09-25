#include "vex.h"

using namespace vex;

int main() {
  vexcodeInit();
  while (TestInertial.isCalibrating()) {
    wait(20, msec);
  }

  double target_inertial = 90;
  double target_rotation2 = 720;
  double uncertainty = 1;
  Drivetrain.setTurnVelocity(15, velocityUnits::pct);

  // Right: robot turns 90 degrees.
  while (fabs(TestInertial.rotation(degrees)) < target_inertial - uncertainty) {
    Drivetrain.turn(right);
  }
  
  Drivetrain.stop(brake);
  controller_screen();
  TestInertial.setRotation(0, degrees);
  wait(500, msec);

  // Left: Robot turns 90 degrees.
  TestInertial.setRotation(0, degrees);
  while (fabs(TestInertial.rotation(degrees)) < target_inertial - uncertainty) {
    Drivetrain.turn(left);
  }

  controller_screen();
  Drivetrain.stop(brake);
  Rotation2.resetPosition();
  wait(500,msec);
  
  //Move forward until Rotation2 reaches the target position.
  while(fabs(Rotation2.position(degrees)) < target_rotation2 - uncertainty){
    Drivetrain.drive(forward);
  }

  controller_screen();
  Drivetrain.stop(brake);
  Rotation2.resetPosition();
  wait(500, msec);

  //Move backward until Rotation2 reaches the target position.
  while(fabs(Rotation2.position(degrees)) < target_rotation2 - uncertainty){
    Drivetrain.drive(reverse);
  }

  controller_screen();
  Drivetrain.stop(brake);

}
