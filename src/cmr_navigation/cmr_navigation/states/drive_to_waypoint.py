import time
from yasmin import State

class DriveToWaypointState(State): 
    def __init__(self): 
        super().__init__(['DriveToWaypoint_Recovery', 'DriveToWaypoint_Done'])
    
    def execute(self, blackboard): 
        print('DriveToWaypoint: Entering DriveToWaypoint State')
        time.sleep(3)
        return 'DriveToWaypoint_Done'