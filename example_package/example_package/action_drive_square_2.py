''' 
action_drive_square_2.py
Tufts Create®3 Educational Robot Example
by Maddie Pero

This is an example of combining two action clients in one class. To learn how to write mulitple classes
in one script reference action_drive_square
'''

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

'''
import messages/actions
'''
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle

class SquareActionClient(Node):
    ''' 
    This is an action client. It sends a goal to an action server which sends
    back feedback and the result of the goal. The goal of this action client is to
    drive and then turn the Create 3 to make a square. Here we are defining a class
    'SquareActionClient' which is a subclass of Node.
    '''
    
    def __init__(self):
        
        '''First we initialize the class by calling the node constructor & 
        naming our node 'sqaure_action_client'
        '''        
        super().__init__('square_action_client')
        
        '''
        Then we initilaize two action clients. One to drive the robot and one to turn it. 
        We include where to add the action client (self), the type of action, and the name
        of the action 
        '''        
        self.drive = ActionClient(self, DriveDistance, '/[Namespace]/drive_distance')
        self.turn = ActionClient(self, RotateAngle, '/[Namespace]/rotate_angle')
  
        '''
        Below we initialize a counter that we will call later so that 
        the robot only drives one square
        '''
        self.i = 0 

    def send_drive(self):
        '''
        this function defines the goal message to drive the robot down one side of the 
        sqaure and then sends the goal to the robot
        '''
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = 0.3
        goal_msg.max_translation_speed = 1.0

        ''' 
        this method waits for a the action server to be available
        '''
        self.drive.wait_for_server()
        
        '''
        then we send the the goal to the server
        '''
        self._send_goal_future = self.drive.send_goal_async(goal_msg)
        
        '''
        Returns a future to a goal handle. We need to register a callback 
        for when the future is complete
        '''
        self._send_goal_future.add_done_callback(self.drive_response_callback)
        
    def send_turn(self):
        '''
        Here we repeat the same process but to turn the corner of the square
        '''
        goal_msg_turn = RotateAngle.Goal()
        goal_msg_turn.angle = 1.57
        goal_msg_turn.max_rotation_speed = 1.0
        
        self.turn.wait_for_server()
        self._send_goal_future = self.turn.send_goal_async(goal_msg_turn)
        self._send_goal_future.add_done_callback(self.turn_response_callback)

    def drive_response_callback(self, future):
        '''
        A callback that is executed when the future is complete.
        The future is completed when an action server accepts or rejects the goal request.
        Since there will be no result, we can check and determine if the goal was rejected
        and return early
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        
        '''
        We can request to see if the goal request was accepted or rejected.
        Future will complete when the result is ready.
        This step is registering a callback (similar to that of the goal response).
        '''
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.drive_result_callback)
        
    def turn_response_callback(self, future):
        ''' 
        Again we repeat the process but for the corner turns
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.turn_result_callback)

    def drive_result_callback(self, future):
        '''
        Here, we are logging the result sequence.
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        
        '''
        then we call our function to make the robot turn the corner
        '''
        print('now turn')
        self.send_turn()
        
    def turn_result_callback(self, future):
        ''' 
        Again we log the result.
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        
        ''' 
        if the counter is less than 3 we want the robot to drive another 
        side and turn again so we call our send_drive() function.
        if the counter is greater than 3 (meaning the robot has driven 
        asll four sides of the sqaure) we shut down the node.
        '''
        if self.i < 3:
        	self.i += 1
        	print('now drive')
        	self.send_drive()
        else:
        	rclpy.shutdown()
        

def main(args=None):
    '''
    Initializes ROS2 and creates an instance of 
    'SquareActionClient'
    '''
    rclpy.init(args=args)
    square_action_client = SquareActionClient() 
    square_action_client.send_drive()
    '''
    Sends the first goal and waits until goal is done & shuts down the node if there is 
    an excception.
    '''
    try:
        rclpy.spin(square_action_client)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        square_action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
