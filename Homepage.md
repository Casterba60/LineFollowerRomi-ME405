# Autonomous Line Following Robot
Cole Sterba, Devon Bolt

Dec. 13 2024

ME405, Cal Poly SLO 
## Overview:
This project is the final product of the ME 405 class at Cal Poly SLO. The project uses a Romi chassis as the basis of an autonomous robot designed to line follow, avoid an obstacle, and return to home. The goal of this project was to follow a predetermined line-following obstacle course as fast as possible that had multiple different line obstacles as well as a physical box obstacle. 
The Romi uses a differential drive system with 2 controllable DC motors. Each wheel has an encoder built in so a Proportional Integral Control loop was established to control each motor. The brains of the robot is a Nucleo-L476RG board with an STM32L476 microcontroller. This connects to the DC motors, the encoders, an IMU, a line sensor array, 6 bump sensors, and the Romi chassis power distribution board. It uses the line sensor array to detect the position of the line, the IMU for heading feedback, and the bump sensors for obstacle detection. 

![](media/Romi.jpg)

*Figure 1: Completed Romi Robot*

## Code:

Romi uses a scheduler based task structure. This ensures that the most important tasks always run on time and the less important tasks don’t get in the way. The highest priority tasks are the motor controllers and encoder feedback updates. In order for the robot to successfully drive forward, the motor position and speed has to be constantly measured and its setpoint has to be updated regularly. If these tasks lag behind, the robot will lose track of how fast it’s going and may overcompensate or never update. The other, lower priority task, is the robot’s finite state machine. 

![](media/FSM.png)

*Figure 2: Romi Finite State Machine*

Romi’s finite state machine has 5 states (see Figure 2) . It starts in the Init state where all the code is initialized. Next it automatically moves to the Idle state. The motors are disabled and all it does is wait for the user button on the Nucleo to be triggered. Once the button is pressed, it enters the Line Following state. In this state, it is constantly reading the position of the line and adjusting its heading to try to keep the line in the center of the robot. By using proportional control, it adjusts the speeds of the left and right motor based on the error in the line position. If the line is to the left of the robot, it increases the left motor speed and decreases the right motor speed by the same amount. This ensures that the center of the robot will always be travelling forward at a constant velocity. If the line sensor does not detect the line, the robot will drive straight. 

Romi will continue to follow the line until it detects that at least one of the bump sensors has been triggered. This triggers a state change to the Obstacle Avoidance state. In this state, it follows a predetermined trajectory to avoid the box obstacle. This trajectory has 6 parts (see Figure 3). The first part is a 78 degree clockwise turn/reverse. To accomplish this, the left wheel is set to 0 speed and the right wheel reverses until the heading is 78 degrees greater than the heading at the very beginning of the track. It was important to use this global heading reference because the heading at which it hit the box was inconsistent. Once the first turn is complete, Romi drives straight for 10 inches using the encoder ticks as feedback. During this stretch its heading is used as the feedback for a proportional control system. Next it makes an 80 degree counterclockwise turn using the same system as the first turn. Then it drives straight for 16 inches to get around the box. After that, it makes another 80 degree counterclockwise turn. After this turn, it is facing towards the line so it is set to drive straight until it detects the line. Once it detects the line, it stops and turns to orient itself with the line again. At this point, a flag is set to show that the obstacle is passed. This concludes the Obstacle Avoidance state so it returns to the Line Following state. 

![](media/obstaclePath.png)

*Figure 3: Obstacle Avoidance Trajectory State Diagram*

After the obstacle has been passed, Romi follows the line until it detects the horizontal finish line. At this point it enters the final state, Return Home (Figure 4). Once in Return Home, it turns to the heading of the finish box, drives straight into the finish box, turns around until its heading is the same as its starting heading and then drives along that heading for a predetermined distance until it reaches the starting box. Once it is almost to the starting box, it ramps down its speed instead of instantly stopping. This reduces the potential for misaligned break drift. This phenomenon occurs when the motors have unequal friction so one stops faster than the other. This causes the robot to turn to one side when it stops which is undesirable. 

![](media/returnHome.png)

*Figure 4: Return Home State Diagram*

## Data Analysis and Tuning

![](media/stepResponse.png)

*Figure 5: Motor Controller Step Response*

The motor controllers for the on-board motors were controlled via a PI controller which took input from the built-in encoders. The gains of this controller were tuned by monitoring the step response of the motor when commanded to 15 rad/s from stop. The starting gains of Kp = 4 and Ki = 6 were obtained through trial and error. New gains were determined through this experimentation, yielding the best step response from a Kp = 7 and a Ki =9, but with little difference varying Ki from 6 to 9. The final iteration of the code used a Ki = 7. This yielded a settling time of about 2 seconds for a 15 rad/s command, but the speed of the initial spike was enough to handle the varying inputs of the line sensor. In the future, we would tune each motor controller separately as they exhibited different parameters and settling times during testing, which required workarounds for performance.

![](media/sensorCal.png)

*Figure 6: Line Sensor Settling Time vs. Reflectance*

The line sensors were calibrated based on the values in figure XXX. Since it is an RC based line sensor, the reflectance/color of a material can be determined by the settling time of the RC circuit. Using Figure XXX, we were able to tune each sensor in the array independently with 4 thresholds to represent a gradient from white to black, with black returning higher values. These thresholds were not evenly spaced, but rather represented the nonlinear behavior of the system. From all 8 sensors, a centroid was calculated from their thresholded readings which represented the position of the line relative to the sensor array on a scale of -1 to 1. This value is multiplied by a gain and then fed to a proportional controller which sets the motor desired speeds based on the error in line position. 

A heading feedback loop using proportional control was used for the states in which Romi needs to drive without a line to follow. This noticeably decreased the drift. Since the motors have different characteristics, their settling times and step responses differ from each other. If they were simply set to the same speed without heading feedback, one would ramp faster than the other leading to unwanted heading change. The heading controller was given a desired heading at the beginning of each trajectory and measured the error was used to adjust the speeds of the motors. This controller could function while Romi was stationary as well as moving, as the motor speed differential was not a function of its linear velocity. 

## Appendix

![](media/wiringtable1.png)
![](media/wiringtable2.png)

*Table 1: Complete Pinout*

![](media/wiringDiagram.png)

*Figure 7: Complete Wiring Diagram*

