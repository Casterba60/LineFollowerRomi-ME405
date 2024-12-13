## @file encoder.py
#  This file is the Romi Robot obstacle detection class, initializing required pins and creating 
#  methods required to properly utilize the 6 bump sensors/limit switches
#
#  The obstacle detection class takes readings across the 6 bump sensors mounted to the front. The @c get_state task
#  reads the state of all of these sensors and returns true if one of them is tripped, indicating a collision. 
# 
#  The obstacle detection class was designed to have the ability to be expanded upon, with the option
#  of using IR or ultrasound sensors to detect an obstacle prior to collision. These changes have yet to 
#  be implemented
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


import pyb #type: ignore

class ObstacleDetection:
    def __init__(self,bumpSensorPins):
        self.bumpPins = []
        for idx, pin in enumerate(bumpSensorPins):
            self.bumpPins.append(pyb.Pin(pin,mode=pyb.Pin.IN,pull=pyb.Pin.PULL_UP))
    def get_state(self):
        for pin in self.bumpPins:
            if(pin.value() == False):
                return True
        return False
