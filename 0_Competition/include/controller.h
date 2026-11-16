#ifndef CONTROLLER_H
#define CONTROLLER_H

// Ouput the Rotation and Inertial sensor values to the controller screen
void controller_screen();

//ArmMotor control functions
void controller_L1_Pressed();
void controller_L2_Pressed();

//Lift control functions (hold R1 = up, hold R2 = down)
void controller_R1_Pressed();
void controller_R2_Pressed();

//Intake control functions (hold X = in, hold Y = out)
void controller_X_Pressed();  // intake forward
void controller_Y_Pressed();  // intake reverse

//Carousel control function (hold B to run)
void controller_B_Pressed();

#endif
