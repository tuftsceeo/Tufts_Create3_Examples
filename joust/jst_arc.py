import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector


from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import DriveArc


class DriveArcActionClient(Node):
    def __init__(self):
    
        super().__init__('drive_arc_action_client')
           
        self._action_client = ActionClient(
            self, DriveArc, 'Ygritte/drive_arc')

    def send_goal(self, angle=3.14, radius=0.3, translate_direction=1, max_translation_speed=0.3):
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

def arc(args=None):
    angle = 3.14
    speed = 0.3   

    rclpy.init(args=args)
    action_client = DriveArcActionClient()

    action_client.send_goal(angle, 0.3, 1, speed)
    #action_client.send_goal()
    rclpy.spin(action_client)
    time.sleep(0.5)  
    
def main(args=None):
    arc()


if __name__ == '__main__':
    main()
