'''!@file encoder.py
@brief 
The encoder object is used to control the encoders on Amtek PG6712A077-R3 Motors
@details

@author Cole Sterba, Devon Bolt
@date October 17, 2024

@ TO DO
need to make the encoder class update function on the scheduler... 



# Encoder Signal |   Recommeneded CPU Pin (any timer can be used)      
#   ENC1_A       |   PB_6             
#   ENC1_B       |   PB_7                   
#   ENC2_A       |   PA_8             
#   ENC2_B       |   PA_9            
 
'''

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

