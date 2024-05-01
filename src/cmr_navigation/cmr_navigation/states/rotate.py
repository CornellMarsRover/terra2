import time
from yasmin import State

class RotateState(State): 
    def __init__(self):
        super().__init__(['Rotate_Rotate', 'Rotate_DriveToPose', 'Rotate_DriveToWaypoint'])

    def execute(self, blackboard): 
        print('Rotate: Entering Rotate State')
        time.sleep(3)
        return 'Rotate_DriveToWaypoint'