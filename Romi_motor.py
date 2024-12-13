# Mechatronics Lab 3
# Cole Sterba Devon Bolt

from pyb import Pin, Timer # type: ignore
import time

class Romi_motor:

    def __init__ (self, EN_Pin, Direction_Pin, Effort_Pin, PWM_tim, tim_channel):
        self.ENABLE = Pin(EN_Pin, mode=Pin.OUT_PP)
        self.EFFORT = Pin(Effort_Pin, mode=Pin.OUT_PP)
        self.DIRECTION = Pin(Direction_Pin, mode=Pin.OUT_PP)
        self.PWM = PWM_tim.channel(tim_channel, mode = Timer.PWM, pin = self.EFFORT)
        self.ENABLE.low()
        self.DIRECTION.low()
        self.PWM.pulse_width_percent(0)
        pass

    def set_duty (self, duty):
        if (-100 <= duty < 0):
            self.PWM.pulse_width_percent(-duty)
            self.DIRECTION.low()
        elif (0 <= duty <= 100):
            self.PWM.pulse_width_percent(duty)
            self.DIRECTION.high()
        else:
            raise ValueError
        pass
    def enable (self):
        self.ENABLE.high()
        #print("enabled")
        pass
    def disable (self):
        self.ENABLE.low()
        #print("disabled")
        pass

 