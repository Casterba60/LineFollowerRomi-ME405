## @file LineSensor.py
#  This file is the Romi Robot Line Sensor class. The Line Sensor is a QTR-8RC Reflectance Sensor.
#  The Class initializes required pins and creates methods required to properly utilize the encoders
#
#  The Line Sensor functionality relies on 2 nested classes. The @c LineSensor class builds out functionality
#  for one individual sensor, where the @c LineSensorArray class combines 8 LineSensor objects to create functionality
#  as a singular object. The LineSensorArray class assumes that the line sensors are in a horizontal line.
#  For the @c LineSensor class, the @c update_value() method is used to trigger the sensor to take 
#  a reading. The @c get_value method returns the most recent reading. The @c LineSensorArray class uses the @c
#  update_line_position() method to update the internal line position value based on the readings from the 8 line sensors
#  This value is between -1 and 1 which represents the position of a line along the array, zero being centered. 
#  In order to do this, readings are linearized and have thresholds applied based on testing done. 
#  The @c get_line_position() method returns this value.
# 
# 
#  @author Cole Sterba, Devon Bolt
#  @date   2024-Nov-12 Approximate date of creation of file
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


import time
from pyb import Pin, ADC # type: ignore 

class LineSensor:
    '''!@brief Class for a single line sensor
    '''
    def __init__(self, Value_pin, LED_pin): 
        """!
        Initializes the LineSensor Object
        @param Value_Pin: A Nucleo pin that will be connected to the analog pin of the line sensor. Output capability
                          to drive the sensor high to reset it and the input to measure how long it takes to decay.
        @param LED_pin:   A Nucleo pin to trigger the LED to turn on. This will be the same for multiple other sensors.
                          Connects to the CTRL EVEN or CTRL ODD pin of the pine sensor array
        """
        self.VALUE_PIN_NUMBER = Value_pin
        self.VALUE_PIN = Pin(Value_pin, mode=Pin.OUT_PP)
        self.LED_PIN = Pin(LED_pin, mode=Pin.OUT_PP)
        self.MAX_DECAY_TIME = 2000
        self.update_value()

    def update_value(self):
        """ 
        reads the analog value of the line sensor
        Step 1: Turn the LED On
        Step 2: Set the Value_Pin to an output and drive it high for at least 10 microseconds
        Step 3: Set the Value_Pin to an input and measure the time it takes to decay in microseconds. Decay time is capped by the internal parameter MAX_DECAY_TIME
        Step 4: Turn LED off
        Step 5: Sets the internal VALUE parameter to the measured decay time
        """
        
        self.LED_PIN.high() # Step 1

        self.VALUE_PIN = Pin(self.VALUE_PIN_NUMBER, mode=Pin.OUT_PP) # Step 2
        self.VALUE_PIN.high()
        # self.VALUE_PIN = ADC(self.VALUE_PIN_NUMBER) # May not need this line

        # Waiting for 10 us
        start = time.ticks_us()
        deadline = time.ticks_add(start, 10) # add 10 usec interval
        while(time.ticks_diff(deadline, time.ticks_us())>=0):
            pass

        # Setting Value Pin to input
        self.VALUE_PIN = Pin(self.VALUE_PIN_NUMBER, mode=Pin.IN) # Step 3

        # Waiting until pin goes low
        start = time.ticks_us()
        decay_time = -1 # if decay_time stays at -1, it can be caught with an error later
        while(self.VALUE_PIN.value()>0):
            decay_time = time.ticks_diff(time.ticks_us(), start) # gets time since start in usec
            if(decay_time>=self.MAX_DECAY_TIME): # if bigger than cutoff time, break
                break
        
        self.VALUE = decay_time # set value to decay time in us
        self.LED_PIN.low() # Step 4

    def get_value(self):
        """
        Updates the value of the sensor and then returns it
        """
        self.update_value()
        # Returns the current value of the line sensor
        return self.VALUE
    

class LineSensorArray:
    '''!@brief Class for an array of line sensor objects
    '''
    def __init__(self, Even_pin, Odd_pin, Pin_list): 
        """!
        Initializes the LineSensorArray object by setting up an array of line sensor object.
        @param Even_pin: Pin to control the even number line sensor array LEDs
        @param Odd_pin: Pin to control the odd number line sensor array LEDs
        @param Pin_List: Ordered list of the analog output pins of the line sensor array
        """
        self.SENSOR_LIST = [LineSensor(Pin_list[0], Even_pin), 
                            LineSensor(Pin_list[1], Odd_pin),
                            LineSensor(Pin_list[2], Even_pin),
                            LineSensor(Pin_list[3], Odd_pin),
                            LineSensor(Pin_list[4], Even_pin),
                            LineSensor(Pin_list[5], Odd_pin),
                            LineSensor(Pin_list[6], Even_pin),
                            LineSensor(Pin_list[7], Odd_pin)]
        self.LINE_POSITION = 0
        self.NUM_SENSORS = 8
        self.ALL_ON_PERCENT = 0.85 # if more than this percent of the sum of the sensor readings, horizontal line hit

    def get_line_position(self):
        """
        Updates the line position using the internal update_line_position() function then returns the value
        """
        self.update_line_position()

        # Under normal operation value between -1 and 1. -1 is left, 1 is right
        # return 2 if all sensors are tripped (for finish detection)
        return self.LINE_POSITION 

    def update_line_position(self):
        """
        Reads the value from all 8 LineSensor objects. 
        Uses custom calibration data to linearize, threshold, and normalize the values for each LineSensor.
        Creates an array with the modified LineSensor readings.
        Finds the centroid of the values and returns that as a value between -1 and 1.
        Negative values represent the line being to the left of the robot and Positive values represent the line being to the right.
        In the case that no line is detected, outputs 0.
        In the event that most of the sensors detect strongly (a horizontal line is detected), outputs 2
        """
        readings = []
        readings_total = 0
        for i in range(self.NUM_SENSORS):
            sensor = self.SENSOR_LIST[i]
            sensor_value = sensor.get_value()
            modified_value = self.threshold_linear(sensor_value, i)
            readings.append(modified_value) # holds values between 0 and 1
            readings_total += modified_value
            # print(f"Getting Sensor {i} value: {sensor_value}")
        
        if(readings_total >= self.NUM_SENSORS*self.ALL_ON_PERCENT):
            self.LINE_POSITION = 2
        else:
            self.LINE_POSITION = self.centroid(readings)

    def threshold_linear(self, sensor_reading, sensor):
        """
        Returns a linearized and thresholded value based on calibration data
        @param sensor_reading: A LineSensor reading (values from 0-MAX_DECAY_TIME)
        @param sensor: the number of the sensor the reading came from. Used for specific calibration.
        """
        if(sensor==0):
            if(sensor_reading<600):
                modified_reading = 0
            elif(sensor_reading<1200):
                modified_reading = 0.5
            elif(sensor_reading<1800):
                modified_reading = 0.75
            else:
                modified_reading = 1
        elif(sensor==1):
            if(sensor_reading<700):
                modified_reading = 0
            elif(sensor_reading<1200):
                modified_reading = 0.5
            elif(sensor_reading<1900):
                modified_reading = 0.75
            else:
                modified_reading = 1
        elif(sensor==2):
            if(sensor_reading<600):
                modified_reading = 0
            elif(sensor_reading<1100):
                modified_reading = 0.5
            elif(sensor_reading<1700):
                modified_reading = 0.75
            else:
                modified_reading = 1
        elif(sensor==3):
            if(sensor_reading<600):
                modified_reading = 0
            elif(sensor_reading<1100):
                modified_reading = 0.5
            elif(sensor_reading<1700):
                modified_reading = 0.75
            else:
                modified_reading = 1
        elif(sensor==4):
            if(sensor_reading<700):
                modified_reading = 0
            elif(sensor_reading<1200):
                modified_reading = 0.5
            elif(sensor_reading<1600):
                modified_reading = 0.75
            else:
                modified_reading = 1
        elif(sensor==5):
            if(sensor_reading<600):
                modified_reading = 0
            elif(sensor_reading<1000):
                modified_reading = 0.5
            elif(sensor_reading<1600):
                modified_reading = 0.75
            else:
                modified_reading = 1
        elif(sensor==6):
            if(sensor_reading<600):
                modified_reading = 0
            elif(sensor_reading<1100):
                modified_reading = 0.5
            elif(sensor_reading<1900):
                modified_reading = 0.75
            else:
                modified_reading = 1
        elif(sensor==7):
            if(sensor_reading<800):
                modified_reading = 0
            elif(sensor_reading<1300):
                modified_reading = 0.5
            elif(sensor_reading<1950):
                modified_reading = 0.75
            else:
                modified_reading = 1

        return modified_reading
    
    def third_order(self, x, a, b, c, d):
        """
        NOTE: UNUSED
        Calculates the result of a third order polynomial. 
        Was going to be used for more specific linearization of the line sensors but ended up being unhelpful.
        """
        return (a*(x**3) + b*(x**2) + c*x + d)


    def linearize(self, value, sensor):
        """
        NOTE: UNUSED
        Linearizes the line sensor readings based on third order approximations of their calibration data.
        Not fully implemented.
        """
        if(sensor==0):
            linear_reading = self.third_order(value, 1.37*(10**(-6)), -5.72*(10**(-3)), 8.32, -2.72*(10**(3)))
        elif(sensor==1):
            linear_reading = self.third_order(value, 1.37*(10**(-6)), -5.72*(10**(-3)), 8.32, -2.72*(10**(3)))
        elif(sensor==2):
            linear_reading = self.third_order(value, 1.37*(10**(-6)), -5.72*(10**(-3)), 8.32, -2.72*(10**(3)))
        elif(sensor==3):
            linear_reading = self.third_order(value, 1.37*(10**(-6)), -5.72*(10**(-3)), 8.32, -2.72*(10**(3)))
        elif(sensor==4):
            linear_reading = self.third_order(value, 1.37*(10**(-6)), -5.72*(10**(-3)), 8.32, -2.72*(10**(3)))
        elif(sensor==5):
            linear_reading = self.third_order(value, 1.37*(10**(-6)), -5.72*(10**(-3)), 8.32, -2.72*(10**(3)))
        elif(sensor==6):
            linear_reading = self.third_order(value, 1.37*(10**(-6)), -5.72*(10**(-3)), 8.32, -2.72*(10**(3)))
        elif(sensor==7):
            linear_reading = self.third_order(value, 1.37*(10**(-6)), -5.72*(10**(-3)), 8.32, -2.72*(10**(3)))

        return linear_reading
        

    def threshold(self, linear_reading):
        """
        NOTE: UNUSED
        Returns a discretized and normalized version of the input linearized reading
        Scale is 0 -> line sensor MAX_DECAY_TIME
        """
        
        time_min = 0
        time_max = self.SENSOR_LIST[0].MAX_DECAY_TIME
        steps = 4
        chunkSize = (time_max-time_min)/steps # size of each bin for thresholding
        block = (int(linear_reading/chunkSize))/steps # by casting to an int, it cuts off decimal
        if block>1:
            return 1
        elif block<0:
            return 0
        return block # discrete value between 0 and 1

    def centroid(self, readings):
        """
        Calculates the centroid of the readings by finding a weighted sum and dividing it by a regular sum of the readings
        Returns a value between -1 and 1.
        @param readings: an array of modified sensor readings
        """
        # returns the centroid of the readings 
        weighted_sum = 0
        sum = 0
        for i in range(self.NUM_SENSORS): 
            weighted_sum += readings[i]*(i-(self.NUM_SENSORS-1)/2)
            sum += readings[i]
        
        if(sum>0):
            return weighted_sum/(sum*3.5) # normalized to be between -1 and 1
        else:
            return 0
