/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       avery                                                     */
/*    Created:      3/3/2026, 1:47:00 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;

vex::motor rightFront = vex::motor(vex::PORT11, vex::gearSetting::ratio18_1, false);
vex::motor leftFront = vex::motor(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor rightRear = vex::motor(vex::PORT19, vex::gearSetting::ratio18_1, false);
vex::motor leftRear = vex::motor(vex::PORT7, vex::gearSetting::ratio18_1, false);
vex::motor rightTop = vex::motor(vex::PORT14, vex::gearSetting::ratio18_1, false);
vex::motor leftTop = vex::motor(vex::PORT6, vex::gearSetting::ratio18_1, false);

motor_group RightDrive = motor_group(rightFront, rightRear, rightTop);
motor_group LeftDrive = motor_group(leftFront, leftRear, leftTop);

inertial Inertial = inertial(PORT5);

vex::smartdrive Drivetrain = vex::smartdrive(RightDrive, LeftDrive, Inertial, 3.25, 3.25, 14.5, distanceUnits::in, (0.75));
// left, right, diameter of wheel, diameter of wheel, distance between wheels, distance units, gear ratio

double kP = 0.5; // proportional gain constant (tune this value for your robot). Bigger = faster gain, Smaller = slower gain. If the robot is oscillating around the target distance, try reducing this value. If the robot is not reaching the target distance, try increasing this value.

double error = 0;          // initialize error to 0
double targetHeading = 0;  // initialize target heading to 0
double currentHeading = 0; // initialize current heading to 0
double motorPower = 0;     // initialize motor power to 0

void turnCW();
void turnCCW();

int main()
{
    wait(5000, msec); //temporary

    Inertial.calibrate(); // calibrate the inertial sensor
    while (Inertial.isCalibrating())
    {
        wait(100, msec);
    }

    targetHeading = 90;                                           // set target heading to 90 degrees
    currentHeading = Drivetrain.heading(vex::rotationUnits::deg); // measure current heading

    error = targetHeading - currentHeading; // calculate error as the difference between target heading and current heading
    motorPower = kP * error;                // calculate motor power using proportional control formula

    if (motorPower > 90) // limit motor power to not burn out the motors
        motorPower = 90;
    if (motorPower < -90)
        motorPower = 90;

    //turnCW();
Drivetrain.setHeading(targetHeading, rotationUnits::deg); // set the target heading for the drivetrain

    while (1)
    {
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}

void turnCW()
{
    LeftDrive.setVelocity(motorPower, percentUnits::pct);
    RightDrive.setVelocity(-motorPower, percentUnits::pct);
    LeftDrive.spin(forward);
    RightDrive.spin(forward);
}

void turnCCW()
{
    LeftDrive.setVelocity(-motorPower, percentUnits::pct);
    RightDrive.setVelocity(motorPower, percentUnits::pct);
   
}
