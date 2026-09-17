/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Mon Aug 24 2020                                           */
/*    Description:  Rotation Sensing                                          */
/*                                                                            */
/*    This program will demonstrate how to use Rotation Sensor commands       */
/*    to get information about the sensor's angle, position, and velocity     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    1, 10           
// Rotation2            rotation      2               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Reset Rotation Sensor position
  Rotation2.resetPosition();
  TestInertial.calibrate();

  wait(1, seconds);

  // Print Rotation Sensor values to the screen in an infinite loop
  // while (true) {
    
    // Clear the screen and set the cursor to the top left corner at the start of each loop
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    Drivetrain.turn(left, 50, velocityUnits::pct);
    wait(1, seconds);
    Drivetrain.stop(coast);
    Controller1.Screen.print("Int: %.2f Rot: %.2f", 
    TestInertial.heading(degrees), Rotation2.position(degrees));
    

  // }
}
