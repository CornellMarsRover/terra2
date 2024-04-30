import time
from yasmin import State

class DriveToPoseState(State): 
    def __init__(self): 
        super().__init__(['DriveToPose_DriveToWaypoint', 'DriveToPose_Recovery', 'DriveToPose_Rotate'])
    
    def execute(self, blackboard):
        print('DriveToPose: Entering DriveToPose State')
        time.sleep(3)
        return 'DriveToPose_DriveToWaypoint'