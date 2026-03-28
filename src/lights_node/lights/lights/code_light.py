import time
from enum import Enum, auto

class LightMode(Enum):
    SOLID = auto()
    FADE = auto()
    BLINK = auto()
    ONE_BY_ONE = auto()

class BrightControl:
    def __init__(self):
        self.mode = LightMode.SOLID
        self.period = 1.0
        self.start_time = time.time()

    def set_mode(self, mode: LightMode, period=None):
        self.mode = mode
        if period is not None:
            self.period = period
        self.start_time = time.time()

    def bright(self):
        elapsed = time.time() - self.start_time
        
        if self.mode == LightMode.SOLID:
            return 255, 255
        
        if self.mode == LightMode.FADE:
            return 0, 0
        
        if self.mode == LightMode.BLINK:
            if (elapsed % self.period) < (self.period / 2):
                return 255, 255  
            else:
                return 0, 0      
        
        if self.mode == LightMode.ONE_BY_ONE:
             if (elapsed % self.period) < (self.period / 2):
                return 255, 0 
             else:
                return 0, 255
        
        return 0, 0