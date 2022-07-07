import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time

from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle

'''
update comments
'''

class DriveDistanceActionClient(Node):
# drive a specific distance

    def __init__(self):
# initialize node
        super().__init__('drive_distance_action_client')
        self._action_client = ActionClient(
            self, DriveDistance, '{Namespace}/drive_distance')

    def send_goal(self, distance=0.5, max_translation_speed=0.15):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.max_translation_speed = max_translation_speed

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
        '''
        Kate add
        '''
        

    def get_result_callback(self, future):
        '''
        Future that will complete when the result is ready.
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

        rclpy.shutdown()
        

class RotateActionClient(Node):
    '''
    rotate 90º
    '''

    def __init__(self):
        '''
initizalize node
        '''
        super().__init__('rotate_action_client')
        self._action_client = ActionClient(self, RotateAngle, 'Ygritte/rotate_angle')

    def send_goal(self, angle=1.57, max_rotation_speed=0.5):
        '''
        sends a goal to the server
        '''
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle 
        goal_msg.max_rotation_speed = max_rotation_speed

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
       '''
       Callback that is executed when an action server accepts or rejects the goal request.
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        '''
kate add
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()
        
def forward(args=None):

    dist  = 0.5
    speed = 0.25

    rclpy.init(args=args)
    action_client1 = DriveDistanceActionClient()
    action_client1.send_goal(dist, speed)
    rclpy.spin(action_client1)
    time.sleep(0.5)
    
def turn(args=None):

    angle = 1.57
    speed = 0.5

    rclpy.init(args=args)
    action_client2 = RotateActionClient()
    action_client2.send_goal(angle, speed)
    rclpy.spin(action_client2)
    time.sleep(0.5)   


def main(args=None):

    for i in range(4):
        forward()
        turn()


if __name__ == '__main__':
    main()
