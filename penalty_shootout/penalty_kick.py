'''
penalty_kick.py
Tufts Create®3 example
by Maddie Pero

This is an example of how to get your Create®3 robot to preform a Paneka penalty kick. 
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

class PenaltyKick1(Node):

    def __init__(self):
    	super().__init__('penalty_kick_1')
    	self.declare_parameter('/JonSnow/motion_control/safety_override','full')
    	self.drive = ActionClient(self, DriveDistance, '/JonSnow/drive_distance')
    	self.turn = ActionClient(self, RotateAngle, '/JonSnow/rotate_angle')
    	self.arc = ActionClient(self, DriveArc, '/JonSnow/drive_arc')
    	self.pos = ActionClient(self, NavigateToPosition, '/JonSnow/navigate_to_position')
    
    def move1(self):
        goal_msg1 = DriveDistance.Goal()
        goal_msg1.distance = -0.25
        goal_msg1.max_translation_speed = 2.0

        self.drive.wait_for_server()
        self._send_goal_future = self.drive.send_goal_async(goal_msg1)
        self._send_goal_future.add_done_callback(self.move1_response_callback)
        
    def move1_response_callback(self, future):
    	goal_handle1 = future.result()
    	if not goal_handle1.accepted:
    		self.get_logger().info('Goal rejected :(')
    		return
    		
    	self.get_logger().info('Goal accepted :)')
    	self._get_result_future = goal_handle1.get_result_async()
    	self._get_result_future.add_done_callback(self.move1_result_callback)

    def move1_result_callback(self, future):
    	result = future.result().result
    	self.get_logger().info('Result: {0}'.format(result))
    	print('now move2')
    	self.move2()
        
    def move2(self):
        goal_msg2 = DriveArc.Goal()
        goal_msg2.translate_direction = 1
        goal_msg2.angle = .57
        goal_msg2.radius = .4
        goal_msg2.max_translation_speed = 2.0
        
        self.arc.wait_for_server()
        self._send_goal_future = self.arc.send_goal_async(goal_msg2)
        self._send_goal_future.add_done_callback(self.move2_response_callback)
        
    def move2_response_callback(self, future):
    	goal_handle2 = future.result()
    	if not goal_handle2.accepted:
    		self.get_logger().info('Goal rejected :(')
    		return
    	self.get_logger().info('Goal accepted :)')
    	self._get_result_future = goal_handle2.get_result_async()
    	self._get_result_future.add_done_callback(self.move2_result_callback)

    def move2_result_callback(self, future):
    	result = future.result().result
    	self.get_logger().info('Result: {0}'.format(result))
    	print('now move3')
    	self.move3()
        
    def move3(self):
        goal_msg3 = RotateAngle.Goal()
        goal_msg3.max_rotation_speed = 2.0
        goal_msg3.angle = -1.0
        
        self.turn.wait_for_server()
        self._send_goal_future = self.turn.send_goal_async(goal_msg3)
        self._send_goal_future.add_done_callback(self.move3_response_callback)
        
    def move3_response_callback(self, future):
    	goal_handle3 = future.result()
    	if not goal_handle3.accepted:
    		self.get_logger().info('Goal rejected :(')
    		return
    	self.get_logger().info('Goal accepted :)')
    	self._get_result_future = goal_handle3.get_result_async()
    	self._get_result_future.add_done_callback(self.move3_result_callback)

    def move3_result_callback(self, future):
    	result = future.result().result
    	self.get_logger().info('Result: {0}'.format(result))
    	print('SHOOT')
    	rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    penalty_kick_1 = PenaltyKick1() 
    penalty_kick_1.move1()

    try:
        rclpy.spin(penalty_kick_1)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
