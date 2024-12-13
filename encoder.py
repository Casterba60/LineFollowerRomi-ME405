## @file encoder.py
#  This file is the Romi Robot encoder class, initializing required pins and creating 
#  methods required to properly utilize the encoders
#
#  Channel A and Channel B are connected to GPIO pins in order to allow the microchip to 
#  read the output signals of the encoders and increment a timer register. Power and ground 
#  are linked to their respective buses. The @c update function runs as a generator function in
#  the scheduler, allowing it to update every 20ms (can be changed). The @c get_delta function 
#  returns the last increment made to the encoder. The @c get_position function returns the current
#  position of the encoder. The @c zero function zeroes the position of the encoder. 
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

from pyb import Pin, Timer #type: ignore
import time

class Encoder:
    '''!@brief Interface with quadrature encoders
    @details
    '''

    def __init__(self, CHA_pin, CHB_pin, Enc_Timer):
        '''!@brief Constructs an encoder object
        @details
        @param CHA: 1st channel of the encoder output, changes state everytime the encoder passes by it
        @param CHB: 2nd channel of the encoder output, changes state everytime the encoder passes by it, 90 degrees out of phase with CHA
        @param TIM: timer to count the encoder ticks
        @param INT_TIM: Timer that instructs the object when to check for a new position
        '''
        self.TIM = Enc_Timer 
        self.CHA = self.TIM.channel(1,pin=CHA_pin, mode=Enc_Timer.ENC_AB)
        self.CHB = self.TIM.channel(2,pin=CHB_pin, mode=Enc_Timer.ENC_AB)
        self.position = 0 # This holds the total number of counts. Gets updated by the update() method
        self.current_pos = 0 # This holds the current value of the encoder
        self.last_pos = 0
        self.delta = 0 # This is used by the update() method and tracks the difference between the current encoder reading and the previous one

    def update(self, tim=None):
        '''!@brief Updates encoder position and delta
        @details
        The delta describes the position of the encoder relative to the previously collected position
        '''
        while 1:
            self.last_pos = self.current_pos
            self.current_pos = self.TIM.counter()
            self.delta = self.current_pos - self.last_pos

            if(self.delta>=32768):
                self.delta-=65536
            elif(self.delta<=-32768):
                self.delta+=65536
            
            self.position+=self.delta
            yield 0

    def get_position(self):
        '''!@brief Gets the most recent encoder position
        @details
        @return
        '''
        return self.position

    def get_delta(self):
        '''!@brief Gets the most recent encoder delta   
        @details
        @return
        '''
        return self.delta

    def zero(self):
        '''!@brief Resets the encoder position to zero
        @details
        '''
        self.position = 0
        self.current_pos = 0
        self.delta = 0

