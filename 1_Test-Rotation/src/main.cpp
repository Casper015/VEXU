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
  
  //Create Controller callback events
  Controller1.ButtonL1.pressed(controller_L1_Pressed);
  Controller1.ButtonL2.pressed(controller_L2_Pressed);
  Controller1.ButtonR1.pressed(controller_R1_Pressed);
  Controller1.ButtonR2.pressed(controller_R2_Pressed);
  wait(15,msec);

  ArmMotor.setStopping(hold);
  ClawMotor.setStopping(hold);
  ArmMotor.setVelocity(60, percent);
  ClawMotor.setVelocity(30, percent);

  wait(1, seconds);

  // Print Rotation Sensor values to the screen in an infinite loop
  // while (true) {
    
    // Clear the screen and set the cursor to the top left corner at the start of each loop
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    // Drivetrain.turn(left, 50, velocityUnits::pct);
    while (true){
      LeftDriveSmart.setVelocity(Controller1.Axis3.position(), percent);
      RightDriveSmart.setVelocity(Controller1.Axis2.position(), percent);
      LeftDriveSmart.spin(forward);
      RightDriveSmart.spin(forward);

      Drivetrain.setTurnVelocity(Controller1.Axis1.position(), percent);
      Drivetrain.turn(right);

      wait(5, msec);
      Controller1.Screen.print("Int: %.2f Rot: %.2f", 
      TestInertial.heading(degrees), Rotation2.position(degrees));
    }

    

  // }
}
