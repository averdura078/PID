/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       avery                                                     */
/*    Created:      3/3/2026, 1:47:00 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "RobotConfig.h"
#include "PID.h"

#include <cmath>

using namespace vex;

int loopDelay = 10; // delay between control loop iterations (in milliseconds)

void driveWithPID(double target)
{
    double targetDistance = target;
    while (true)
    {
        // 0.75 is correct
        double gearRatio = 0.75;

        double leftDegrees = leftTop.position(rotationUnits::deg);
        double rightDegrees = rightTop.position(rotationUnits::deg);
        double currentMotorRotation = (leftDegrees + rightDegrees) / 2.0;

        // convert motor degrees to wheel degrees
        double wheelRotation = currentMotorRotation * gearRatio;

        // convert wheel degrees to distance traveled by the robot
        double currentDistance = (M_PI * 3.25) * wheelRotation / 360.0; // 3.25 is the diameter of the wheel, so this formula calculates the distance traveled by the robot based on how much the wheel has rotated.
        // The formula is circumference * (π * diameter) * (rotation in degrees / 360), which gives us the distance traveled in inches.

        // P
        double error = targetDistance - currentDistance; // calculate error as the difference between target distance and current distance

        // D
        double previousError = error;              // save the current error for the next loop iteration
        double derivative = error - previousError; // compute derivative as the change in error since the last loop iteration.

        // I
        double integral = 0;                      // initialize integral to 0
        integral += error * (loopDelay / 1000.0); // compute integral as the accumulation of error as time passes (in seconds)
                                                  // clamp integral term to prevent integral windup (when the integral term accumulates too much error and causes the robot to overshoot the target)
        if (integral > 200)
            integral = 200;
        if (integral < -200)
            integral = -200;

        // calculate motor power using PID control formula
        double motorPower = dP * error + dI * integral + dD * derivative;

        // clamp to prevent motor burnout (don't let motors spin at more than 90% power)
        if (motorPower > 90)
            motorPower = 90;
        if (motorPower < -90)
            motorPower = -90;

        // make the robot spin
        LeftDrive.spin(forward, motorPower, percentUnits::pct);
        RightDrive.spin(forward, motorPower, percentUnits::pct);

        if (fabs(error) <= 0.1)
        { // stop condition
            LeftDrive.stop();
            RightDrive.stop();
            break;
        }

        // Use this for troubleshooting. May use brain screen instead and check other values too.
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Current Rotation: %f", currentMotorRotation);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Error: %f", error);

        // allow other tasks to run
        this_thread::sleep_for(loopDelay);
    }
}

void turnWithPID(double target)
{
    double targetRotation = target;
    while (true)
    {
        double currentRotation = Drivetrain.rotation(vex::rotationUnits::deg); // measure current drivetrain rotation (uses inertial sensor)

        // P
        double error = targetRotation - currentRotation; // calculate error as the difference between target rotation and current rotation

        // D
        double previousError = error;                                       // save the current error for the next loop iteration
        double derivative = (error - previousError) / (loopDelay / 1000.0); // compute derivative as the change in error over the change in timesince the last loop iteration.

        // I
        double integral = 0;                      // initialize integral to 0
        integral += error * (loopDelay / 1000.0); // compute integral as the accumulation of error as time passes (in seconds)
                                                  // clamp integral term to prevent integral windup (when the integral term accumulates too much error and causes the robot to overshoot the target)
        if (integral > 200)
            integral = 200;
        if (integral < -200)
            integral = -200;

        // calculate motor power using PID control formula
        double motorPower = tP * error + tI * integral + tD * derivative;

        // clamp to prevent motor burnout (don't let motors spin at more than 90% power)
        if (motorPower > 90)
            motorPower = 90;
        if (motorPower < -90)
            motorPower = -90;

        // make the robot spin
        LeftDrive.spin(forward, motorPower, percentUnits::pct);
        RightDrive.spin(forward, -motorPower, percentUnits::pct);

        // prevent oscillation around the target
        if (fabs(error) <= 0.3) // stop if the error is less than or equal to 0.3 degrees
        {
            LeftDrive.stop();
            RightDrive.stop();
            break; // break out of the while loop if close enough
        }

        // Use this for troubleshooting. May use brain screen instead and check other values too.
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Current Rotation: %f", currentRotation);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Error: %f", error);

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
        // turnWithPID(90);  // turn 90 degrees
        driveWithPID(10); // drive forward 10 inches
        break;            // use break if you want the robot to stop after reaching the target distance. Remove break if you want the robot to keep trying to reach the target distance (in case it gets pushed off course or something).
    }
}
