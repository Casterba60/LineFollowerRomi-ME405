## @file controller.py
#  This file is the Romi Robot motor controller, implementing a PI controller based
#  on the output from the built in encoders
#
#  the @c run function is the main component of the controller class, which takes readings
#  from the encoder as well as time passed to calculate proportional and integral error 
#  over time. These errors are multiplied by their respective gains to get the output for 
#  motor duty cycle. the @c setSpeed function allows the alteration of the desired speed from 
#  which error is calculated.  
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
import pyb #type: ignore
import utime #type: ignore

class controller:
    """!@brief PI controller to drive the motors on the Romi Motor"""

    def __init__ (self, motor, encoder,ENC_PERIOD):
        self.motor = motor
        self.encoder = encoder
        self.motor.enable()
        self.ENC_PERIOD = ENC_PERIOD
        self.refSpeed = 0.0

    def run(self):
        '''Run the PI motor controller using input from encoder'''
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
        '''Set the desired speed for the controller
        @param desiredSpeed the speed the motor should drive to'''
        self.refSpeed = desiredSpeed

    def getEncoderPos(self):
        '''Return the raw encoder position from the encoder obj'''
        return self.encoder.get_position()

        