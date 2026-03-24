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
vex::controller Controller;
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

double tP = 0.4; // proportional gain constant (tune this value for your robot). Bigger kP = faster gain, Smaller kP= slower gain. If the robot is oscillating around the target distance, try reducing this value. If the robot is not reaching the target distance, try increasing this value.
/*NOTES ABOUT PROPORTIONAL CONTROL
- proportional = kP * error (this means the product of the proportional gain constant and the error)
- the proportional term reacts proportionally to the error, meaning the bigger the error, the stronger the motor power.
- proportional control is the main component of a PID controller, and is responsible for getting the robot to the target distance
*/
double tD = 0.1; // derivative gain constant (tune this value for your robot). Bigger kD = stronger braking effect. Smaller kD = weaker breaking effect.
/*NOTES ABOUT DERIVATIVE CONTROL
- derivative = d(error)/dt (this means the derivative of the error over the derivative of the time, or the change in error over the change in time)
- this is simplified to derivative = (error - previousError) / ∆t (this means the change in error over the change in time. Change in time is how big the delay in your loop is. If your loop runs every 10 milliseconds, then ∆t is 0.01 seconds.)
- this is further simplified to derivative = error - previousError (this means the change in error, or how much the error has changed since the last time we calculated it)
- we tune kD to accomodate for the fact that we are not dividing by ∆t. If our loop runs every 10 milliseconds, then we multiply our derivative by 100 (because we are not dividing by 0.01)
- derivative is big and negative when the robot gets close to a target, reducing motor power.
- derivative is positive when error is increasing, meaning the robot is pushed away from the target.
- derivative prevents overshooting and oscillation around the target.
*/
double tI = 0.1; // integral gain constant (tune this value for your robot).
/*NOTES ABOUT INTEGRAL CONTROL
- integral is the accumulation of error over time, meaning it is the sum of all past errors.
- integral += error * ∆t (this means integral equals current value of integral plus the product of the current error multiplied by how much time passed in seconds).
- this is simplified to integral += error * (loopDelay / 1000) (this means integral equals current value of integral plus the product of the current error and the loop delay in seconds).
- if you program the robot to spin to 90 degrees but it stops at 89.5:
- put simply, the integral “remembers” that the robot has been behind the target for a while, gradually adding more power until the error disappears.
*/

double dP = 0.5;
double dI = 0.3;
double dD = 0.01;

double motorPower = 0; // initialize motor power to 0

double error = 0;         // initialize error to 0
double previousError = 0; // initialize previous error to 0
double derivative = 0;    // initialize derivative to 0
double integral = 0;      // initialize integral to 0

double targetRotation = 90; // initialize target rotation to 0
double currentRotation = 0; // initialize current rotation to 0

double targetDistance = 0;  // initialize target distance to 0
double currentDistance = 0; // initialize current distance to 0

void driveWithPID(double target)
{
    targetDistance = target;
    while (true)
    {
        double leftDegrees = leftFront.position(rotationUnits::deg);
        double rightDegrees = rightFront.position(rotationUnits::deg);
        double averageDegrees = (leftDegrees + rightDegrees) / 2.0;

        currentDistance = (M_PI * 3.25) * averageDegrees / 360.0;

        // Use this for troubleshooting. May use brain screen instead and check other values too.
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Current Rotation: %f", currentRotation);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Error: %f", error);

        // P
        error = targetDistance - currentDistance; // calculate error as the difference between target distance and current distance

        // D
        derivative = error - previousError; // compute derivative as the change in error since the last loop iteration.
        previousError = error;              // save the current error for the next loop iteration

        // I
        integral += error * (loopDelay / 1000.0); // compute integral as the accumulation of error as time passes (in seconds)
        // clamp integral term to prevent integral windup (when the integral term accumulates too much error and causes the robot to overshoot the target)
        //  if (integral > 200)
        //      integral = 200;
        //  if (integral < -200)
        //      integral = -200;

        // calculate motor power using PID control formula
        motorPower = dP * error + dI * integral + dD * derivative;

        // clamp to prevent motor burnout (don't let motors spin at more than 90% power)
        if (motorPower > 90)
            motorPower = 90;
        if (motorPower < -90)
            motorPower = -90;

        // make the robot spin
        LeftDrive.spin(forward, motorPower, percentUnits::pct);
        RightDrive.spin(forward, motorPower, percentUnits::pct);

        // uncomment the following lines if necessary to further prevent oscillation around the target
        // if (fabs(error) <= 0.5) //stop if the error is less than or equal to 0.5 degrees
        // {
        //     LeftDrive.stop();
        //     RightDrive.stop();
        //     break;
        // }

        // allow other tasks to run
        this_thread::sleep_for(loopDelay);
    }
}

void turnWithPID(double target)
{
    targetRotation = target;
    while (true)
    {
        // Use this for troubleshooting. May use brain screen instead and check other values too.
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Current Rotation: %f", currentRotation);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Error: %f", error);

        currentRotation = Drivetrain.rotation(vex::rotationUnits::deg); // measure current drivetrain rotation (uses inertial sensor)

        // P
        error = targetRotation - currentRotation; // calculate error as the difference between target rotation and current rotation

        // D
        derivative = error - previousError; // compute derivative as the change in error since the last loop iteration.
        previousError = error;              // save the current error for the next loop iteration

        // I
        integral += error * (loopDelay / 1000.0); // compute integral as the accumulation of error as time passes (in seconds)
        // clamp integral term to prevent integral windup (when the integral term accumulates too much error and causes the robot to overshoot the target)
        //  if (integral > 200)
        //      integral = 200;
        //  if (integral < -200)
        //      integral = -200;

        // calculate motor power using PID control formula
        motorPower = tP * error + tI * integral + tD * derivative;

        // clamp to prevent motor burnout (don't let motors spin at more than 90% power)
        if (motorPower > 90)
            motorPower = 90;
        if (motorPower < -90)
            motorPower = -90;

        // make the robot spin
        LeftDrive.spin(forward, motorPower, percentUnits::pct);
        RightDrive.spin(forward, -motorPower, percentUnits::pct);

        // uncomment the following lines if necessary to further prevent oscillation around the target
        // if (fabs(error) <= 0.5) //stop if the error is less than or equal to 0.5 degrees
        // {
        //     LeftDrive.stop();
        //     RightDrive.stop();
        //     break;
        // }

        // allow other tasks to run
        this_thread::sleep_for(loopDelay);
    }
}

int main()
{
    Inertial.calibrate(); // calibrate the inertial sensor
    while (Inertial.isCalibrating())
    {
        wait(100, msec);
    }

    while (1)
    {
        driveWithPID(18); // drive forward 1 meter (39 inches)
        // break;
    }
}
