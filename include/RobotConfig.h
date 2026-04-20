// DECLARE YOUR CONTROLLER, BRAIN, MOTORS, SENSORS, AND OTHER DEVICES HERE

#include "vex.h"
#include <cmath>

extern vex::controller Controller;
extern vex::brain Brain;

extern vex::motor rightFront;
extern vex::motor rightRear;
extern vex::motor rightTop;

extern vex::motor leftFront;
extern vex::motor leftRear;
extern vex::motor leftTop;

extern vex::motor_group RightDrive;
extern vex::motor_group LeftDrive;

extern vex::inertial Inertial; //inertial sensor

extern vex::smartdrive Drivetrain;

extern bool killSwitchActivated; // flag to track if kill switch was activated