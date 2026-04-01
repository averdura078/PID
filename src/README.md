Read this document to learn the basics of PID control, what is going on in this project, and how to implement it into your own project.


// proportional gain constant (tune this value for your robot). Bigger kP = faster gain, Smaller kP= slower gain. If the robot is oscillating around the target distance, try reducing this value. If the robot is not reaching the target distance, try increasing this value.
/*NOTES ABOUT PROPORTIONAL CONTROL
- proportional = kP * error (this means the product of the proportional gain constant and the error)
- the proportional term reacts proportionally to the error, meaning the bigger the error, the stronger the motor power.
- proportional control is the main component of a PID controller, and is responsible for getting the robot to the target distance
*/

// derivative gain constant (tune this value for your robot). Bigger kD = stronger braking effect. Smaller kD = weaker breaking effect.
/*NOTES ABOUT DERIVATIVE CONTROL
- derivative = d(error)/dt (this means the derivative of the error over the derivative of the time, or the change in error over the change in time)
- this is simplified to derivative = (error - previousError) / ∆t (this means the change in error over the change in time. Change in time is how big the delay in your loop is. If your loop runs every 10 milliseconds, then ∆t is 0.01 seconds.)
- derivative is big and negative when the robot gets close to a target, reducing motor power.
- derivative is positive when error is increasing, meaning the robot is pushed away from the target.
- derivative prevents overshooting and oscillation around the target.
*/

// derivative gain constant (tune this value for your robot). Bigger kD = stronger braking effect. Smaller kD = weaker breaking effect.
/*NOTES ABOUT DERIVATIVE CONTROL
- derivative = d(error)/dt (this means the derivative of the error over the derivative of the time, or the change in error over the change in time)
- this is simplified to derivative = (error - previousError) / ∆t (this means the change in error over the change in time. Change in time is how big the delay in your loop is. If your loop runs every 10 milliseconds, then ∆t is 0.01 seconds.)
- derivative is big and negative when the robot gets close to a target, reducing motor power.
- derivative is positive when error is increasing, meaning the robot is pushed away from the target.
- derivative prevents overshooting and oscillation around the target.
*/