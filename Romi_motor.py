## @file Romi_motor.py
#  This file is the Romi motor class, initializing required pins and creating 
#  methods required to properly drive the motors 
#
#  The motors are controlleed via 3 pins, effort, enable, and direction. The effort
#  pin drives the speed of the motor, and takes a PWM input. The enable pin turns 
#  the motor on and off. The direction pin takes a digital signal and controls the 
#  direction of the motor. The @c set_duty method inputs the PWM signal for the 
#  effort pin and the sign drives the DIR pin. @c enable and @c disable control the 
#  control the enable pin. 
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
        '''Sets the duty cycle of the motor'''
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
        '''Enables the motor (EN pin high)'''
        self.ENABLE.high()
        #print("enabled")
        pass
    def disable (self):
        '''Disables the motor (EN pin low)'''
        self.ENABLE.low()
        #print("disabled")
        pass

 