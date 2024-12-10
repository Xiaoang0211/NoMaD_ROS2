import rclpy
from rclpy.time import Time
from rclpy.clock import Clock

class ROSData:
    def __init__(self, timeout: int = 3, queue_size: int = 1, name: str = ""):
        self.clock = Clock()
        self.timout = timeout
        self.last_time_received = self.clock.now()
        self.queue_size = queue_size
        self.data = None
        self.name = name
        self.phantom = False
    
    def get(self):
        return self.data
    
    def set(self, data): 
        current_time = self.clock.now()
        time_waited = (current_time - self.last_time_received).nanoseconds / 1e9
        
        if self.queue_size == 1:
            self.data = data
        else:
            if self.data is None or time_waited > self.timout: # reset queue if timeout
                self.data = []
            if len(self.data) == self.queue_size:
                self.data.pop(0)
            self.data.append(data)
        self.last_time_received = current_time
        
    def is_valid(self, verbose: bool = False):
        if self.data == None and verbose:
            return False
        
        time_waited = (self.clock.now() - self.last_time_received).nanoseconds / 1e9
        valid =  time_waited < self.timout
        if self.queue_size > 1:
            valid = valid and len(self.data) == self.queue_size
        if verbose and not valid:
            print(f"Not receiving {self.name} data for {time_waited} seconds (timeout: {self.timout} seconds)")
        return valid