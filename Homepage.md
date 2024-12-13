#Autonomous Line Following Robot

##Overview:
This project is the final product of the ME 405 class at Cal Poly SLO. The project uses a Romi chassis as the basis of an autonomous robot designed to line follow, avoid an obstacle, and return to home. The goal of this project was to follow a predetermined line-following obstacle course as fast as possible that had multiple different line obstacles as well as a physical box obstacle. 
The Romi uses a differential drive system with 2 controllable DC motors. Each wheel has an encoder built in so a Proportional Integral Control loop was established to control each motor. The brains of the robot is a Nucleo-L476RG board with an STM32L476 microcontroller. This connects to the DC motors, the encoders, an IMU, a line sensor array, 6 bump sensors, and the Romi chassis power distribution board. It uses the line sensor array to detect the position of the line, the IMU for heading feedback, and the bump sensors for obstacle detection. 
