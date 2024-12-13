# Mechantronics 405 Final Project State Machine
# Cole Sterba, Devon Bolt

import time
import pyb #type: ignore
import utime #type: ignore

class statemachine:

    def __init__ (self,left_controller,right_controller,IMU,buttonStatus,lineArray,obstacleDetection):
        self.debug = False
        self.left_controller = left_controller
        self.right_controller = right_controller
        self.imu = IMU
        self.buttonStatus = buttonStatus
        self.lineArray = lineArray
        self.obstacleDetection = obstacleDetection
        self.gain = -15 
        self.global_x = 0
        self.global_y = 0
        



    def FSM(self):
        nominalSpeed = 5; #rad/s
        obstaclePassed = False
        state = 0
        while(1):
            #state 0 init
            if(state == 0):
                state = 1
                yield

            #state 1 idle
            if(state == 1):
                if(self.buttonStatus.get(in_ISR=False) == True):
                    state = 2
                    self.buttonStatus.put(False,in_ISR=False)
                    self.starting_Heading = self.imu.get_heading()
                    if(self.debug): print("Running")
                    if(self.debug): print(self.starting_Heading)
                    startCount = 0
                yield
        
            #state 2 line follow
            if(state == 2):
                CF = self.lineArray.get_line_position()*self.gain
                while(startCount < 22): 
                    CF = 0
                    startCount += 1
                if(CF == 2*self.gain): 
                    if(obstaclePassed == True):
                        state = 4 # return home 
                        if(self.debug): print("Going Home")
                    else:
                        CF = 0
                        self.right_controller.setSpeed(-(nominalSpeed-CF))
                        self.left_controller.setSpeed(-(nominalSpeed+CF))   
                else: 
                    #apply speed increase to appropriate wheel CF = corrective factor
                    self.right_controller.setSpeed(-(nominalSpeed-CF))
                    self.left_controller.setSpeed(-(nominalSpeed+CF))
                    #check for obstacle
                if(self.obstacleDetection.get_state() and not obstaclePassed):
                    state = 3 # avoid the obstacle
                    self.previousHeading = self.starting_Heading # sets the initial heading of the obstacle avoidance path
                    self.totalHeadingChange = 0
                    if(self.debug): print("avoiding obstacle")
                yield

            # state 3 collision avoidance 
            if(state == 3):
                substate = 1
                straight_speed = 10
                while(substate > 0):
                    if(substate == 1): #78 degree CW reversal in place
                        turn_angle = 78
                        self.currentHeading = self.imu.get_heading()
                        self.deltaHeading = self.currentHeading-self.previousHeading
                        self.previousHeading = self.currentHeading
                        if(self.deltaHeading < -180): self.deltaHeading += 360
                        self.totalHeadingChange += self.deltaHeading
                        if(self.totalHeadingChange < turn_angle):
                            self.right_controller.setSpeed(10) # speeds in rad/s, need to test and tune
                            self.left_controller.setSpeed(0)
                        else:
                            self.previousPosition = self.left_controller.getEncoderPos()
                            self.totalHeadingChange = 0
                            self.totalPosChange = 0
                            self.straightHeading = self.imu.get_heading()
                            substate = 2
                            if(self.debug): print("First Turn Finished")

                    if(substate == 2): #straight line for 10 inches (1663 encoder ticks)
                        distance = 1663
                        self.currentPosition = self.left_controller.getEncoderPos()
                        self.deltaPosition = self.currentPosition-self.previousPosition
                        self.previousPosition = self.currentPosition
                        if(self.deltaPosition > 32768): self.deltaPosition -= 65536
                        self.totalPosChange += self.deltaPosition
                        if(self.totalPosChange > -distance):
                            #self.right_controller.setSpeed(straight_speed*1.1) # speeds in rad/s, need to test and tune
                            #self.left_controller.setSpeed(straight_speed)
                            self.headingControl(self.straightHeading,straight_speed)
                        else:
                            self.previousHeading = self.imu.get_heading()
                            self.totalHeadingChange = 0
                            self.totalPosChange = 0
                            substate = 3 
                            if(self.debug): print("First Straight Finished")

                    if(substate == 3): #80 degree CCW turn in place
                        turn_angle = -80
                        self.currentHeading = self.imu.get_heading()
                        self.deltaHeading = self.currentHeading-self.previousHeading
                        self.previousHeading = self.currentHeading
                        if(self.deltaHeading > 180): self.deltaHeading -= 360
                        self.totalHeadingChange += self.deltaHeading
                        if(self.totalHeadingChange > turn_angle):
                            self.right_controller.setSpeed(-10) # speeds in rad/s, need to test and tune
                            self.left_controller.setSpeed(0)
                        else:
                            self.previousHeading = self.imu.get_heading()
                            self.totalHeadingChange = 0
                            self.totalPosChange = 0
                            self.straightHeading = self.imu.get_heading()
                            substate = 4
                            if(self.debug): print("Second Turn Finished")

                    if(substate == 4): #straight line for 16 in (2660 encoder ticks)
                        distance = 2660
                        self.currentPosition = self.left_controller.getEncoderPos()
                        self.deltaPosition = self.currentPosition-self.previousPosition
                        self.previousPosition = self.currentPosition
                        if(self.deltaPosition > 32768): self.deltaPosition -= 65536
                        self.totalPosChange += self.deltaPosition
                        if(self.totalPosChange > -distance):
                            #self.right_controller.setSpeed(straight_speed) # speeds in rad/s, need to test and tune
                            #self.left_controller.setSpeed(straight_speed*1.2)
                            self.headingControl(self.straightHeading,straight_speed)
                        else:
                            self.previousHeading = self.imu.get_heading()
                            self.totalHeadingChange = 0
                            self.totalPosChange = 0
                            substate = 5 
                            if(self.debug): print("Second Straight Finished")

                    if(substate == 5): # 80 CCW degree turn in place
                        turn_angle = -80
                        self.currentHeading = self.imu.get_heading()
                        self.deltaHeading = self.currentHeading-self.previousHeading
                        self.previousHeading = self.currentHeading
                        if(self.deltaHeading > 180): self.deltaHeading -= 360
                        self.totalHeadingChange += self.deltaHeading
                        if(self.totalHeadingChange > turn_angle):
                            # Drive back up circle
                            self.right_controller.setSpeed(-10) # speeds in rad/s, need to test and tune
                            self.left_controller.setSpeed(0)
                        else:
                            self.totalHeadingChange = 0
                            self.totalPosChange = 0
                            self.straightHeading = self.imu.get_heading()
                            substate = 6

                    if(substate == 6): #drive straight until line is detected
                        if(self.lineArray.get_line_position() == 0): 
                            self.headingControl(self.straightHeading,straight_speed)
                        else:
                            self.previousHeading = self.imu.get_heading()
                            self.totalHeadingChange = 0
                            self.totalPosChange = 0
                            substate = 7
                            if(self.debug): print("Third Straight Finished")
                        
                    if(substate == 7): #60 CW degree turn in place
                        turn_angle = 60
                        self.currentHeading = self.imu.get_heading()
                        self.deltaHeading = self.currentHeading-self.previousHeading
                        self.previousHeading = self.currentHeading
                        if(self.deltaHeading < -180): self.deltaHeading += 360
                        self.totalHeadingChange += self.deltaHeading
                        if(self.totalHeadingChange < turn_angle):
                            self.right_controller.setSpeed(10) # speeds in rad/s, need to test and tune
                            self.left_controller.setSpeed(-10)
                        else:
                            self.previousPosition = self.left_controller.getEncoderPos()
                            self.totalHeadingChange = 0
                            self.totalPosChange = 0
                            substate = 0
                            state = 2
                            obstaclePassed = True
                            if(self.debug): print("Fourth Turn Finished")
                    yield
                yield

            #state 4 return home
            if(state == 4):
                substate = 1
                desiredHeading = 0 
                while(substate > 0):
                    if(substate ==1): #correct heading
                        if(desiredHeading == 0):
                            desiredHeading = self.starting_Heading + 180
                            if(desiredHeading > 360): desiredHeading -= 360
                            if(self.debug): print(desiredHeading)
                        #if(self.headingControl(desiredHeading,0)):
                            #if(self.debug): print("First Alignment")
                        self.previousPosition = self.left_controller.getEncoderPos()
                        self.totalPosChange = 0    
                        substate = 2
                    if(substate == 2):
                        distance = 700
                        self.currentPosition = self.left_controller.getEncoderPos()
                        self.deltaPosition = self.currentPosition-self.previousPosition
                        self.previousPosition = self.currentPosition
                        if(self.deltaPosition > 32768): self.deltaPosition -= 65536
                        self.totalPosChange += self.deltaPosition
                        if(self.totalPosChange > -distance): 
                            self.headingControl(desiredHeading,straight_speed)
                        else:
                            self.totalPosChange = 0
                            if(self.debug): print("Second Alignment")
                            substate = 3
                    if(substate == 3):
                        if(self.headingControl(self.starting_Heading,0,1/12)):
                            substate = 4
                            self.previousPosition = self.left_controller.getEncoderPos()
                            if(self.debug): print("Third Alignment")
                    if(substate == 4):
                        distance = 7100
                        self.currentPosition = self.left_controller.getEncoderPos()
                        self.deltaPosition = self.currentPosition-self.previousPosition
                        self.previousPosition = self.currentPosition
                        if(self.deltaPosition > 32768): self.deltaPosition -= 65536
                        self.totalPosChange += self.deltaPosition
                        if(self.totalPosChange > -0.9*distance): 
                            self.headingControl(self.starting_Heading,2*straight_speed,0.61)
                        elif(self.totalPosChange > -0.96*distance):
                            self.headingControl(self.starting_Heading,0.75*straight_speed,1/3)
                        elif(self.totalPosChange > -distance):
                            self.headingControl(self.starting_Heading,0.25*straight_speed,1/3)
                        else:
                            self.totalPosChange = 0
                            if(self.debug): print("Fourth Alignment")
                            substate = 0
                            self.right_controller.setSpeed(0)
                            self.left_controller.setSpeed(0)
                    yield
                state = 1
                obstaclePassed = False
                yield
            yield

    def headingControl(self,desiredHeading,velocity,gain=1/15):
        error = desiredHeading - self.imu.get_heading()
        if(error > 180): error -= 360
        if(error < -180): error += 360
        output = gain*error 
        self.right_controller.setSpeed(-(velocity-output))
        self.left_controller.setSpeed(-(velocity+output))
        if(abs(error) < 0.35): return True
        else: return False