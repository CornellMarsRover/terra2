
import time
from yasmin import State

class DriveToStartState(State):
    def __init__(self): 
        super().__init__(['DriveToStart_Rotate'])

    def execute(self, blackboard): 
        print('DriveToStart: Entering DriveToStart State')
        time.sleep(3)
        return 'DriveToStart_Rotate'