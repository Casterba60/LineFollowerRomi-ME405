#Mechatronics405 Line Sensor Calibration Script
#Cole Sterba, Devon Bolt
#This script takes data as the line sensor moves over multiple gradients

import pyb #type: ignore
from LineSensor import LineSensor


lineSensor_Pin = pyb.Pin.cpu.C14 
LED_Pin = pyb.Pin.cpu.C13
lineSensor1 = LineSensor(lineSensor_Pin,LED_Pin)

sensorData = [] #list of each reading
while 1:
    user_input = input("Waiting for next shade... Enter 'q' to end collection")
    if user_input == "q":
        break
    sensorData.append(lineSensor1.get_value())
print(sensorData)
