# Mechantronics 405 PI controller class
# Cole Sterba Devon Bolt

#NOTES:
#Add derivative control

import time
import pyb #type: ignore
import utime #type: ignore

class controller:

    def __init__ (self, motor, encoder,ENC_PERIOD):
        self.motor = motor
        self.encoder = encoder
        self.motor.enable()
        self.ENC_PERIOD = ENC_PERIOD
        self.refSpeed = 0.0

    def run(self):
        Kp = 7 #proportional motor gain
        Ki = 7 #integral motor gain
        integral_error = 0 #integral error
        etime = utime.ticks_ms()
        while 1:
            if(self.refSpeed == 0): 
                self.motor.disable()
                integral_error = 0 
            else:
                self.motor.enable()
            stime = utime.ticks_ms()
            timePassed = utime.ticks_diff(stime, etime)
            etime = utime.ticks_ms()
            self.measuredSpeed = self.encoder.get_delta()*4.3633/self.ENC_PERIOD #convert to rad/s
            error = (self.refSpeed - self.measuredSpeed)
            integral_error += error*float(timePassed/1000)
            L = Kp*error + Ki*integral_error #proportional integral controller
            if L > 100: L = 100
            if L < -100: L = -100
            self.motor.set_duty(L)
            yield 0
    
    def setSpeed(self,desiredSpeed):
        self.refSpeed = desiredSpeed

    def getEncoderPos(self):
        return self.encoder.get_position()

        