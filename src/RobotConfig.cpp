// DEFINE YOUR CONTROLLER, BRAIN, MOTORS, SENSORS, AND OTHER DEVICES HERE

#include "RobotConfig.h"

vex::controller Controller;
vex::brain Brain;

vex::motor rightFront = vex::motor(vex::PORT11, vex::gearSetting::ratio18_1, false); 
vex::motor rightRear = vex::motor(vex::PORT19, vex::gearSetting::ratio18_1, false); 
vex::motor rightTop = vex::motor(vex::PORT14, vex::gearSetting::ratio18_1, true); 
vex::motor leftFront = vex::motor(vex::PORT1, vex::gearSetting::ratio18_1, true); 
vex::motor leftRear = vex::motor(vex::PORT7, vex::gearSetting::ratio18_1, true); 
vex::motor leftTop = vex::motor(vex::PORT6, vex::gearSetting::ratio18_1, false); 

vex::motor_group RightDrive = vex::motor_group(rightFront, rightRear, rightTop); 
vex::motor_group LeftDrive = vex::motor_group(leftFront, leftRear, leftTop); 

vex::inertial Inertial = vex::inertial(vex::PORT5);

vex::smartdrive Drivetrain = vex::smartdrive(RightDrive, LeftDrive, Inertial, 3.25, 3.25, 14.5, vex::distanceUnits::in, (0.75)); 
// left, right, diameter of wheel, diameter of wheel, distance between wheels, distance units, gear ratio

