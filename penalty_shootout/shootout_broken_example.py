'''
shootout_broken_example.py
Tufts Create®3 Educational Robot Example
by Maddie Pero

Edit this code to move your Create®3 soccer player in order to fake out the opponent and score (or save) a penalty kick
'''

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

'''
import messages/actions
'''
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import DriveArc
from irobot_create_msgs.action import NavigateToPosition

class PenaltyKick(Node):
    def __init__(self):
        super().__init__('penalty_kick')
        self.drive = ActionClient(self, DriveDistance, '/[Namespace]/drive_distance')
        self.turn = ActionClient(self, RotateAngle, '/[Namespace]/rotate_angle')
        self.arc = ActionClient(self, DriveArc, '/[Namespace]/drive_arc')
        self.pos = ActionClient(self, NavigateToPosition, '/[Namespace]/navigate_to_position')
    
    def move1(self):
      '''
      In this function you should 
      1. define the goal message
      2. send the goal to a server
      3. set up a goal handle to get the results back when the future is complete
      '''
    def move1_response_callback(self, future):
      '''
      In this function you should
      1. check to see if the goal was accepted or rejected.
      2. log that information if you wish
      '''
    def move1_result_callback(self, future):
      ''' 
      In this function you should
      1. get the result of the action.
      2. log the information if you wish. 
      3. figure out how to preform another move or shutdown.
      '''
def main(args=None):
    '''
    In this function you should
    1. initialize rclpy & your node
    2. determine how you wish to call your classes or functions to preform a set of moves
    '''
    rclpy.init(args=args)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
