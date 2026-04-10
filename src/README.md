Read this document to learn the basics of PID control, what is going on in this project, how to implement it into your own project, and how to tune your PID.



Basics of PID Control
A PID is used to accurately get your robot to a target distance or angle. There are 3 components of a PID: Proportional, Integral, and Derivative.
At the most basic level, the amount of power your motors receive depends on 3 things: how far you are from the target (P), how fast you’re approaching it (D), and how far off you’ve been over time (I).

Simple Math: 
motorPower = P + I + D

- Proportional control
    - Responsible for getting the robot to the target (P is the main component of the PID)
    - Based on size of error
- Integral control
    - Responsible for eliminating leftover error over time
    - Based on the sum of past errors
- Derivative control
    - Responsible for "braking" (preventing overshoot of a target)
    - Based on how quickly the error is changing



Actual Math (you will see this code logic / general structure in this code):
- We define kP, kI, and kD as the proportional, integral, and derivative constants we will adjust in tuning
- The proportional, integral, and derivative terms of motorPower are obtained by multiplying the thing each term depends on by the respective constant
- We add these 3 terms to get motorPower: 
motorPower = proportional term + integral term + derivative term 
motorPower = kP * error + kI * integral + kD * derivative;

How each component is calculated:
P
- proportional = error 
- proportional term = kP * error (this means the product of the proportional constant and the error).
- the proportional term reacts to the error: bigger error = more motor power

Example of proportional function
If the robot is far from the target, it moves fast. As it approaches, it begins going slower. The farther it is, the more power it gets.

I
- integral += error * ∆t (this means integral equals current value of integral plus the product of current error and time passed IN SECONDS).
- integral term = kI * integral
- the integral term reacts to the robot being slightly behind a target for a period of time, and as it accumulates, more power is given to eliminate the error.

Example of integral function
If you program the robot to spin to 90 degrees but it stops at 89.5, the integral “remembers” that the robot has been behind the target for a while, gradually adding more power until the error disappears.

D
- derivative = d(error)/dt (this means the derivative of the error over the derivative of the time, or the change in error over the change in time). Simplified: derivative = (error - previousError) / ∆t (Note: Change in time IN SECONDS).
- derivative term = kD * derivative
- the derivative term reacts to the robot going too fast as it approaches the target and slows it down, preventing it from going too far.

Example of derivative function:
If you program the robot to spin to 90 degrees, and it is moving too quickly as it approaches, the derivative “brakes” to prevent overshooting.



How to Implement
1. 



Step-By-Step Tuning Checklist
1. Set kI and kD to 0.
2. Increase kP until the robot slightly oscillates around the target.
3. Increase kD to dampen the oscillation.
4. Add a tiny amount of kI to fix steady-state error (if it stops just short of the target).

Tips for Troubleshooting / Perfecting Constants
kP
Bigger kP = faster gain, Smaller kP = slower gain. 
If the robot is oscillating around the target distance, try reducing this value. If the robot is not reaching the target distance, try increasing this value.

kD
Bigger kD = stronger braking effect. Smaller kD = weaker braking effect.
If the robot overshoots the target, try increasing kD. If it stops too early, try decreasing kD.

kI
Bigger kI = stronger correction for leftover error. Smaller kI = weaker correction.
If the robot consistently stops short of the target, try increasing kI. If it overshoots or oscillates, try decreasing kI.


