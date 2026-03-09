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
vex::motor rightRear = vex::motor(vex::PORT19, vex::gearSetting::ratio18_1, false);
vex::motor rightTop = vex::motor(vex::PORT14, vex::gearSetting::ratio18_1, true);

vex::motor leftFront = vex::motor(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor leftRear = vex::motor(vex::PORT7, vex::gearSetting::ratio18_1, true);
vex::motor leftTop = vex::motor(vex::PORT6, vex::gearSetting::ratio18_1, false);

motor_group RightDrive = motor_group(rightFront, rightRear, rightTop);
motor_group LeftDrive = motor_group(leftFront, leftRear, leftTop);

inertial Inertial = inertial(PORT5);

vex::smartdrive Drivetrain = vex::smartdrive(LeftDrive, RightDrive, Inertial, 3.25, 3.25, 14.5, distanceUnits::in, (0.75));
// left, right, diameter of wheel, diameter of wheel, distance between wheels, distance units, gear ratio

double kP = 0.5; // proportional gain constant (tune this value for your robot). Bigger = faster gain, Smaller = slower gain. If the robot is oscillating around the target distance, try reducing this value. If the robot is not reaching the target distance, try increasing this value.

double error = 0;          // initialize error to 0
double targetHeading = 0;  // initialize target heading to 0
double currentHeading = 0; // initialize current heading to 0
double motorPower = 0;     // initialize motor power to 0

int main()
{
    wait(5000, msec); // temporary
    
    targetHeading = 90;

    Inertial.calibrate(); // calibrate the inertial sensor
    while (Inertial.isCalibrating())
    {
        wait(100, msec);
    }

    while (1)
    {                                           // set target heading to 90 degrees
        currentHeading = Inertial.heading(deg); // measure current heading

        error = targetHeading - currentHeading; // calculate error as the difference between target heading and current heading
        motorPower = kP * error;                // calculate motor power using proportional control formula

        // clamp
        if (motorPower > 100)
            motorPower = 100;
        if (motorPower < -100)
            motorPower = -100;

        // stop when close
        if (fabs(error) < 1)
        {
            LeftDrive.stop();
            RightDrive.stop();
        }

        // //take shortest path
        // if (error > 180)
        //     error -= 360;
        // if (error < -180)
        //     error += 360;

        LeftDrive.spin(forward, motorPower, pct);
        RightDrive.spin(forward, -motorPower, pct);

        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}