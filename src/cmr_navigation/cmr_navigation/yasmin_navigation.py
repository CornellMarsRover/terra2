#Potential working file for the navigation node using Yasmin
import rclpy

from enum import Enum

from simple_node import Node

from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

from cmr_navigation.states.idle import IdleState
from cmr_navigation.states.drive_to_pose import DriveToPoseState
from cmr_navigation.states.drive_to_start import DriveToStartState
from cmr_navigation.states.drive_to_waypoint import DriveToWaypointState
from cmr_navigation.states.rotate import RotateState
from cmr_navigation.states.recovery import RecoveryState


class NavigationNode(Node): 
    def __init__(self): 
        super().__init__('yasmin_node')
        sm = StateMachine(outcomes = ['Done'])
        sm.add_state('Idle', IdleState(), transitions = {'Idle_DriveToStart': 'DriveToStart'})
        sm.add_state('DriveToStart', DriveToStartState(), transitions = {'DriveToStart_Rotate': 'Rotate'})

        sm.add_state('Rotate', RotateState(), transitions = {'Rotate_Rotate': 'Rotate', 
                                                        'Rotate_DriveToPose': 'DriveToPose', 
                                                        'Rotate_DriveToWaypoint': 'DriveToWaypoint'})
        
        sm.add_state('DriveToPose', DriveToPoseState(), transitions = {'DriveToPose_DriveToWaypoint': 'DriveToWaypoint',
                                                                  'DriveToPose_Recovery': 'Recovery', 
                                                                  'DriveToPose_Rotate': 'Rotate'})
        
        sm.add_state('DriveToWaypoint', DriveToWaypointState(), transitions = {'DriveToWaypoint_Recovery': 'Recovery',
                                                                          'DriveToWaypoint_Done': 'Done'})
        
        sm.add_state('Recovery', RecoveryState(), transitions = {'Recovery_DriveToPose': 'DriveToPose',
                                                            'Recovery_DriveToWaypoint': 'DriveToWaypoint'})

        YasminViewerPub(self, 'YASMIN_DEMO', sm)
        outcome = sm()
        print(outcome)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()