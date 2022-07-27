import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import DriveArc
from irobot_create_msgs.action import WallFollow
from builtin_interfaces.msg import Duration

distance = 10

radius = (distance/75) + 0.16
print(radius)
translate_direction=1
max_translation_speed=0.3
namespace = 'Ygritte'
counter = 0

class DriveArcActionClient(Node):
    def __init__(self):
    
        super().__init__('drive_arc_action_client')
           
        self._action_client = ActionClient(
            self, DriveArc, namespace + '/drive_arc')

    def send_goal(self, angle, radius, translate_direction, max_translation_speed):
        goal_msg = DriveArc.Goal()
        goal_msg.angle = angle
        goal_msg.max_translation_speed = max_translation_speed
        goal_msg.radius = radius
        goal_msg.translate_direction = translate_direction

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


    def get_result_callback(self, future):

        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

        rclpy.shutdown()
        
class WallFollowActionClient(Node):
    def __init__(self):
        super().__init__('wall_follow_action_client')
        self.subscription = self.create_subscription(HazardDetectionVector, namespace + '/hazard_detection', self.listener_callback, qos_profile_sensor_data)
        self._action_client = ActionClient(self, WallFollow, namespace + '/wall_follow')


    def listener_callback(self, msg):
        for detection in msg.detections:
            det = detection.header.frame_id
            if det != "base_link":
               print(det)
               if det == "bump_right":
                   self.send_goal(follow_side=-1)
               elif det == "bump_left":
                   self.send_goal(follow_side=1)
               elif det == "bump_front_right":
                   self.send_goal(follow_side=-1)
               elif det == "bump_front_left":
                   self.send_goal(follow_side=1)
               elif det == "bump_front_center":
                   pass
     
 
    def send_goal(self, follow_side=1, max_runtime=10):
        print('Ready for Action')
         
        goal_msg = WallFollow.Goal()
        goal_msg.follow_side = follow_side
            
        goal_msg.max_runtime = Duration(sec=10, nanosec=0)
         
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def arc1(args=None):
    angle = 3.14

    rclpy.init(args=args)
    action_client = DriveArcActionClient()

    action_client.send_goal(angle, radius, translate_direction, max_translation_speed)
    rclpy.spin(action_client)

def arc2(args=None):
    angle = -3.14
    
    rclpy.init(args=args)
    action_client = DriveArcActionClient()

    action_client.send_goal(angle, radius, translate_direction, max_translation_speed)
    rclpy.spin(action_client)
    
def wall(args=None):
    rclpy.init(args=args)
     
    action_client = WallFollowActionClient()
    action_client.send_goal(follow_side=1, max_runtime=10)
    rclpy.spin(action_client)

    

def main(args=None):
    arc1()
    arc2()
    wall()


if __name__ == '__main__':
    main()
