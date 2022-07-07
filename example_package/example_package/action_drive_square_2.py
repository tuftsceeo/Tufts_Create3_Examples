import rclpy
from rclpy.action import ActionClient

from rclpy.node import Node

from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle

import time 

class SquareActionClient(Node):

    def __init__(self):
        super().__init__('square_action_client')
        self.drive = ActionClient(self, DriveDistance, '/JonSnow/drive_distance')
        self.turn = ActionClient(self, RotateAngle, '/JonSnow/rotate_angle')
        self.i = 0 

    def send_drive(self):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = 0.3
        goal_msg.max_translation_speed = 1.0

        self.drive.wait_for_server()
        self._send_goal_future = self.drive.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.drive_response_callback)
        
    def send_turn(self):
    	goal_msg_turn = RotateAngle.Goal()
    	goal_msg_turn.angle = 1.57
    	goal_msg_turn.max_rotation_speed = 1.0
    	
    	self.turn.wait_for_server()
    	self._send_goal_future = self.turn.send_goal_async(goal_msg_turn)
    	self._send_goal_future.add_done_callback(self.turn_response_callback)

    def drive_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.drive_result_callback)
        
    def turn_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.turn_result_callback)

    def drive_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        print('now turn')
        self.send_turn()
        
    def turn_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        if self.i < 3:
        	self.i += 1
        	print('now drive')
        	self.send_drive()
        else:
        	rclpy.shutdown()
        

def main(args=None):
    rclpy.init(args=args)
    square_action_client = SquareActionClient() 
    square_action_client.send_drive()
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
