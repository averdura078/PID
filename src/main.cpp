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

int loopDelay = 10; // delay between control loop iterations (in milliseconds)

vex::motor rightFront = vex::motor(vex::PORT11, vex::gearSetting::ratio18_1, false);
vex::motor rightRear = vex::motor(vex::PORT19, vex::gearSetting::ratio18_1, false);
vex::motor rightTop = vex::motor(vex::PORT14, vex::gearSetting::ratio18_1, true);

vex::motor leftFront = vex::motor(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor leftRear = vex::motor(vex::PORT7, vex::gearSetting::ratio18_1, true);
vex::motor leftTop = vex::motor(vex::PORT6, vex::gearSetting::ratio18_1, false);

motor_group RightDrive = motor_group(rightFront, rightRear, rightTop);
motor_group LeftDrive = motor_group(leftFront, leftRear, leftTop);

inertial Inertial = inertial(PORT5);

vex::smartdrive Drivetrain = vex::smartdrive(RightDrive, LeftDrive, Inertial, 3.25, 3.25, 14.5, distanceUnits::in, (0.75));
// left, right, diameter of wheel, diameter of wheel, distance between wheels, distance units, gear ratio

double kP = 0.45; // proportional gain constant (tune this value for your robot). Bigger kP = faster gain, Smaller kP= slower gain. If the robot is oscillating around the target distance, try reducing this value. If the robot is not reaching the target distance, try increasing this value.
double kD = 0.3; // derivative gain constant (tune this value for your robot). Bigger kD = stronger braking effect. Smaller kD = weaker breaking effect.
/*NOTES ABOUT DERIVATIVE CONTROL
- derivative = d(error)/dt (this means the derivative of the error over the derivative of the time, or the change in error over the change in time)
- this is simplified to derivative = (error - previousError) / ∆t (this means the change in error over the change in time. Change in time is how big the delay in your loop is. If your loop runs every 10 milliseconds, then ∆t is 0.01 seconds.)
- this is further simplified to derivative = error - previousError (this means the change in error, or how much the error has changed since the last time we calculated it)
- we tune kD to accomodate for the fact that we are not dividing by ∆t. If our loop runs every 10 milliseconds, then we multiply our derivative by 100 (because we are not dividing by 0.01)
- derivative is big and negative when the robot gets close to a target, reducing motor power.
- derivative is positive when error is increasing, meaning the robot is pushed away from the target.
- derivative prevents overshooting and oscillation around the target.
*/

double motorPower = 0;     // initialize motor power to 0
double error = 0;          // initialize error to 0
double previousError = 0; // initialize previous error to 0
double derivative = 0;     // initialize derivative to 0

double targetRotation = 0;  // initialize target rotation to 0
double currentRotation = 0; // initialize current rotation to 0

int main()
{
    Inertial.calibrate(); // calibrate the inertial sensor
    while (Inertial.isCalibrating())
    {
        wait(100, msec);
    }

    targetRotation = 90; // set target heading to 90 degrees

    while (1)
    {
        currentRotation = Drivetrain.rotation(vex::rotationUnits::deg); // measure current rotation

        error = targetRotation - currentRotation; // calculate error as the difference between target rotation and current rotation
        derivative = error - previousError;       // compute derivative as the change in error since the last loop iteration.
        previousError = error;                   // save the current error for the next loop iteration

        motorPower = kP * error + kD * derivative; // calculate motor power using PID control formula

        // clamp to prevent motor burnout (don't let motors spin at more than 90% power)
        if (motorPower > 90)
            motorPower = 90;
        if (motorPower < -90)
            motorPower = -90;

        LeftDrive.spin(forward, motorPower, percentUnits::pct);
        RightDrive.spin(forward, -motorPower, percentUnits::pct);

        // uncomment the following lines to further prevent oscillation around the target
        // if (fabs(error) <= 0.5) //stop if the error is less than or equal to 0.5 degrees
        // {
        //     LeftDrive.stop();
        //     RightDrive.stop();
        // }

        // allow other tasks to run 
        this_thread::sleep_for(loopDelay);
    }
}
