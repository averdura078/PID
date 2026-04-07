// DEFINE YOUR CONTROLLER, BRAIN, MOTORS, SENSORS, AND OTHER DEVICES HERE

#include "RobotConfig.h"

using namespace vex;

controller Controller;
brain Brain;

motor rightFront = motor(PORT11, gearSetting::ratio18_1, false); 
motor rightRear = motor(PORT19, gearSetting::ratio18_1, false); 
motor rightTop = motor(PORT14, gearSetting::ratio18_1, true); 

motor leftFront = motor(PORT1, gearSetting::ratio18_1, true); 
motor leftRear = motor(PORT7, gearSetting::ratio18_1, true); 
motor leftTop = motor(PORT6, gearSetting::ratio18_1, false); 

motor_group RightDrive = motor_group(rightFront, rightRear, rightTop); 
motor_group LeftDrive = motor_group(leftFront, leftRear, leftTop); 

inertial Inertial = inertial(PORT5);

smartdrive Drivetrain = smartdrive(RightDrive, LeftDrive, Inertial, 3.25, 3.25, 14.5, vex::distanceUnits::in, (0.75)); 
// left, right, diameter of wheel, diameter of wheel, distance between wheels, distance units, gear ratio

