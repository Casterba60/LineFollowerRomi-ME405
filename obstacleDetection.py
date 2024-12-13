# Obstacle Detection Class
# Cole Sterba, Devon Bolt

#start with bump sensors, add in IR if needed/wanted

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
