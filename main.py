## @file main.py
#  This file is the main file, containing all needed dependencies, initializing all
#  required objects as well as the scheduler
#
#  This program is built on the micropython library. All required Romi objects, which 
#  include: 2 motors, 2 encoders, 2 motor controllers, 1 IR sensor array containing 
#  8 IR sensors, a 6 sensor bump sensor array, and 1 IMU are instantiated in this file. 
#  The details of each of these objects is detailed in their respective files
#  These objects are all intregated into a set of tasks which run on the scheduler.
#  These tasks are: The overacrching Finite State Machine, updating the encoder positions,
#  and running the individual motor controllers 
# 
#  @author Cole Sterba, Devon Bolt
#  @date   2024-Nov-25 Approximate date of creation of file
#  @date   2024-Dec-12 Final tuning completed
#  @copyright This program is copyright (c) 2024 by C Sterba and D Bolt and
#             released under the GNU Public License, version 3.0.
# 
#  It is intended for educational use only, but its use is not limited thereto.
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

#Import necessary libraries
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

#Constants
ENCPERIOD = 20 #ms
CONTPERIOD = 20 #ms
FSMPERIOD = 40 #ms
#Shares
buttonStatus = task_share.Share("H",name="button status",thread_protect = True)
initialHeading = task_share.Share("H",name="initial heading",thread_protect = True)
currentHeading = task_share.Share("H",name="current heading",thread_protect=True)
#position shares? maybe a list of all 3 values?

def test_imu(imu):
    '''Function to test output of IMU, unused in final program'''
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
    '''Read IMU calibration data from the sensor and write to a .txt file'''
    data = imu.read_calibration_data()  # reading calibration data from imu into array

    with open("IMU_CAL_DATA.txt", "w") as file:
        for item in data:
            #print(f"{item}")
            file.write(f"{item}\n")  # Write each item followed by a newline

def write_imu_cal(imu):
    '''Write IMU calibration data from a .txt file to the sensor'''
    # Read the data from the file
    data = []
    with open("IMU_CAL_DATA.txt", "r") as file:
        for line in file:
            #print(f"{line}")
            data.append(line.strip())  
    
    # write to the IMU
    imu.write_calibration_data(data)


def updateButton(pin):
    '''read onboard button status and update share'''
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

    #Create tasks with generator functions
    UpdateLeftEncoderTask = cotask.Task(left_Encoder.update,name="Update Left Encoder", priority=1, period=ENCPERIOD,
                        profile=True, trace=False)
    UpdateRightEncoderTask = cotask.Task(right_Encoder.update,name="Update Right Encoder", priority=1, period=ENCPERIOD,
                        profile=True, trace=False)
    RightMotorController = cotask.Task(right_Controller.run,name="Right Controller", priority=1, period=CONTPERIOD,profile=True,trace=True)
    LeftMotorController = cotask.Task(left_Controller.run,name="Left Controller", priority=1, period=CONTPERIOD,profile=True,trace=True)
    FSM = cotask.Task(romi_obj.FSM,name="FSM control",priority=0,period=FSMPERIOD,profile=True,trace=False)

    #Append Tasks to IMU
    cotask.task_list.append(UpdateRightEncoderTask)
    cotask.task_list.append(UpdateLeftEncoderTask)
    cotask.task_list.append(RightMotorController)
    cotask.task_list.append(LeftMotorController)
    cotask.task_list.append(FSM)
    #run garbage collector
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
    