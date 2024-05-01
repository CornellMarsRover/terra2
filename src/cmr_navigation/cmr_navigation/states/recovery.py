import time
from yasmin import State

class RecoveryState(State): 
    def __init__(self):
        super().__init__(['Recovery_DriveToPose', 'Recovery_DriveToWaypoint'])
    
    def execute(self, blackboard): 
        print('Recovery: Entering Recovery State')
        time.sleep(3)
        return 'Recovery_DriveToPose'