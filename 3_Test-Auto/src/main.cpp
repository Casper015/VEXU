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

  // Print Rotation Sensor values to the screen in an infinite loop
  while (true) {
    
    // Clear the screen and set the cursor to the top left corner at the start of each loop
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    
    Brain.Screen.print("Angle: ");
    Brain.Screen.print("%.2f", Rotation2.angle());
    Brain.Screen.newLine();

    Brain.Screen.print("Position in Degrees: ");
    Brain.Screen.print("%.2f", Rotation2.position(deg));
    Brain.Screen.newLine();

    Brain.Screen.print("Position in Turns: ");
    Brain.Screen.print("%.2f", Rotation2.position(turns));
    Brain.Screen.newLine();

    Brain.Screen.print("Velocity in RPM: ");
    Brain.Screen.print("%.2f", Rotation2.velocity(rpm));
    Brain.Screen.newLine();

    Brain.Screen.print("Velocity in DPS: ");
    Brain.Screen.print("%.2f", Rotation2.velocity(dps));
    Brain.Screen.newLine();

    // A brief delay to allow text to be printed without distortion or tearing
    wait(0.2, seconds);
  }
}
