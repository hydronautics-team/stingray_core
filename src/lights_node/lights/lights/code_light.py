import time

class BrightControl:
    def __init__(self):
        self.mode = "SOLID"
        self.period = 1.0
        self.start_time = time.time()

    def set_mode(self, mode: str, period=None):
        self.mode = mode
        if period is not None:
            self.period = period
        self.start_time = time.time()

    def bright(self):
        elapsed = time.time() - self.start_time
        
        if self.mode == "SOLID":
            return 255, 255
        
        if self.mode == "FADE":
            return 0, 0
        
        if self.mode == "BLINK":
            if (elapsed % self.period) < (self.period / 2):
                return 255, 255  
            else:
                return 0, 0      
        
        if self.mode == "ONE_BY_ONE":
             if (elapsed % self.period) < (self.period / 2):
                return 255, 0 
             else:
                return 0, 255
        
        return 0, 0