'''!@file 
main.py
@brief 
This project aimed to follow a line while avoiding physical obstacles and returning to home at the finish
@details

@authors
Cole Sterba, Devon Bolt
@date last modified 
December 9, 2024

# L6206 Signal   | Morpho Pin Number | CPU Pin
#   EN_A         |  CN10-33          | PA_10   
#   IN1_A        |  CN10-27          | PB_4          
#   IN2_A        |  CN10-29          | PB_5
#   EN_B         |  CN7-36           | PC_1
#   IN1_B        |  CN7-28           | PA_0
#   IN2_B        |  CN7-30           | PA_1

# Encoder Signal |   CPU Pin              
#   ENC1_A       |   PB_6             
#   ENC1_B       |   PB_7                   
#   ENC2_A       |   PA_8             
#   ENC2_B       |   PA_9            
 
'''

'''LIBRARIES'''
import pyb #type: ignore
import time
import gc

from encoder import Encoder
from Romi_motor import Romi_motor
from controller import controller
from statemachine import statemachine
from obstacleDetection import ObstacleDetection
from LineSensor import LineSensorArray

import cotask
import task_share 
from BNO055 import BNO055


# these 2 lines allow the terminal to output detailed exception 
import micropython #type: ignore
micropython.alloc_emergency_exception_buf(100)

'''CONSTANTS'''
ENCPERIOD = 20 #ms
CONTPERIOD = 20 #ms
FSMPERIOD = 40 #ms
'''SHARES'''
buttonStatus = task_share.Share("H",name="button status",thread_protect = True)
initialHeading = task_share.Share("H",name="initial heading",thread_protect = True)
currentHeading = task_share.Share("H",name="current heading",thread_protect=True)
#position shares? maybe a list of all 3 values?

def test_imu(imu):

    # Set the IMU to NDOF mode (full sensor fusion mode)
    imu._set_mode(imu.NDOF_MODE)

    # Allow some time for the sensor to stabilize
    time.sleep(0.5)

    for i in range(100):
        # Read and print the heading angle
        heading = imu.get_heading()
        print("Heading:", heading)

        # Example of reading other data
        euler_angles = imu.get_euler_angles()
        print("Euler Angles:", euler_angles)

        angular_velocity = imu.get_angular_velocity()
        print("Angular Velocity:", angular_velocity)

        time.sleep(0.1)

def read_imu_cal(imu):
    data = imu.read_calibration_data()  # reading calibration data from imu into array

    with open("IMU_CAL_DATA.txt", "w") as file:
        for item in data:
            #print(f"{item}")
            file.write(f"{item}\n")  # Write each item followed by a newline

def write_imu_cal(imu):
    # Read the data from the file
    data = []
    with open("IMU_CAL_DATA.txt", "r") as file:
        for line in file:
            #print(f"{line}")
            data.append(line.strip())  
    
    # write to the IMU
    imu.write_calibration_data(data)


def updateButton(pin):
    #read button status and store in a share
    buttonStatus.put(True,in_ISR=True)

if __name__ == '__main__':

    # Initializing Motors
    tim_3 = pyb.Timer(3, freq = 20_000) # sets up timer 3 for the right motor
    tim_4 = pyb.Timer(4, freq = 20_000) # sets up timer 4 for the left motor
    right_Motor = Romi_motor(pyb.Pin.cpu.C0, pyb.Pin.cpu.A4, pyb.Pin.cpu.B0, tim_3, 3) # enable, direction, effort, timer, channel
    left_Motor = Romi_motor(pyb.Pin.cpu.B5, pyb.Pin.cpu.B3, pyb.Pin.cpu.B6, tim_4, 1) # enable, direction, effort, timer, channel
    
    # Initializing Encoders
    tim_1 = pyb.Timer(1,period=65535,prescaler=0)
    tim_2 = pyb.Timer(2,period=65535,prescaler=0)
    left_Encoder = Encoder(pyb.Pin.cpu.A8, pyb.Pin.cpu.A9, tim_1) # channel A, channel B, timer
    right_Encoder = Encoder(pyb.Pin.cpu.A0, pyb.Pin.cpu.A1,tim_2) # channel A, channel B, timer

    # Initializing Controllers
    left_Controller = controller(left_Motor,left_Encoder,ENCPERIOD)
    right_Controller = controller(right_Motor,right_Encoder,ENCPERIOD)

    # Initializing Button
    button = pyb.ExtInt(pyb.Pin.cpu.C13, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_NONE, updateButton)

    # Initializing Line Sensor Array
    lineArray = LineSensorArray(pyb.Pin.cpu.A15, 
                                pyb.Pin.cpu.B2, 
                                [pyb.Pin.cpu.C12, 
                                 pyb.Pin.cpu.D2, 
                                 pyb.Pin.cpu.C10,  
                                 pyb.Pin.cpu.C11, 
                                 pyb.Pin.cpu.B13, 
                                 pyb.Pin.cpu.B14, 
                                 pyb.Pin.cpu.B15, 
                                 pyb.Pin.cpu.B1])

    # Initializing Bump Sensor Pins: 
    obstacleDetector = ObstacleDetection([pyb.Pin.cpu.A5,  pyb.Pin.cpu.A6,  pyb.Pin.cpu.A7, 
                                          pyb.Pin.cpu.C5, pyb.Pin.cpu.B11, pyb.Pin.cpu.B12])

    # Initializing IMU
    i2c_bus = 1
    imu = BNO055(i2c_bus)
    write_imu_cal(imu) 
    
    # Initializing state machine
    romi_obj = statemachine(left_Controller,right_Controller,imu,buttonStatus,lineArray,obstacleDetector)

    #zero encoders
    left_Encoder.zero()
    right_Encoder.zero()

    #test_imu(imu)

    UpdateLeftEncoderTask = cotask.Task(left_Encoder.update,name="Update Left Encoder", priority=1, period=ENCPERIOD,
                        profile=True, trace=False)
    UpdateRightEncoderTask = cotask.Task(right_Encoder.update,name="Update Right Encoder", priority=1, period=ENCPERIOD,
                        profile=True, trace=False)
    RightMotorController = cotask.Task(right_Controller.run,name="Right Controller", priority=1, period=CONTPERIOD,profile=True,trace=True)
    LeftMotorController = cotask.Task(left_Controller.run,name="Left Controller", priority=1, period=CONTPERIOD,profile=True,trace=True)
    FSM = cotask.Task(romi_obj.FSM,name="FSM control",priority=0,period=FSMPERIOD,profile=True,trace=False)

    cotask.task_list.append(UpdateRightEncoderTask)
    cotask.task_list.append(UpdateLeftEncoderTask)
    cotask.task_list.append(RightMotorController)
    cotask.task_list.append(LeftMotorController)
    cotask.task_list.append(FSM)

    gc.collect()

    # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    print("Initialized")
    while True:
        try:
            cotask.task_list.pri_sched()

        except KeyboardInterrupt:
            break

    right_Motor.disable()
    left_Motor.disable()
    