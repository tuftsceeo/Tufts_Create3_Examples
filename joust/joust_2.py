import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector


from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import DriveArc

class ArcTurnClient(Node):

    def __init__(self):
        print(11)
      
        super().__init__('arcturn_action_client')
        print(12)
              
        self.arc = ActionClient(self, HazardDetectionVector, 'Ygritte/hazard_detection')
        print(13)
        self.turn = ActionClient(self, RotateAngle, 'Ygritte/rotate_angle')
        print(14)

        self.i = 0 
        
    def listener_callback(self, msg):
        print('1')
        for detection in msg.detections:
            det = detection.header.frame_id
            if det != "base_link":
                print(det)
                if det == "bump_right":
                    self.send_turn(angle=0.25)
                elif det == "bump_left":
                    self.send_turn(angle=-0.25)
                elif det == "bump_front_left":
                    self.send_turn(angle=-1.3)
                elif det == "bump_front_right":
                    self.send_turn(angle=1.3)
                elif det == "bump_front_center":
                    self.send_turn(angle=1.57)


    def send_arc(self, angle=3.14, radius=0.3, translate_direction=1, max_translation_speed=0.3):
        print('2')
        angle=3.14
        radius=0.3
        translate_direction=1
        max_translation_speed=0.3
        goal_msg = DriveArc.Goal()
        goal_msg.angle = angle
        goal_msg.max_translation_speed = max_translation_speed
        goal_msg.radius = radius
        goal_msg.translate_direction = translate_direction

        self._action_client.wait_for_server()
        

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def send_turn(self, angle=3.14, max_rotation_speed=0.5):
        print('3')
        
        goal_msg_turn = RotateAngle.Goal()
        goal_msg_turn.angle = angle
        goal_msg_turn.max_rotation_speed = max_rotation_speed

        self.turn.wait_for_server()
        self._send_goal_future = self.turn.send_goal_async(goal_msg_turn)
        self._send_goal_future.add_done_callback(self.turn_response_callback)

    def arc_response_callback(self, future):
        print('4')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.arc_result_callback)
        
    def turn_response_callback(self, future):
        print('5')

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.turn_result_callback)

    def arc_result_callback(self, future):
        print('6')
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        
        self.send_turn()
        
    def turn_result_callback(self, future):
        print('7')

        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        while flag:
            self.send_arc()
        rclpy.shutdown()
        

def main(args=None):
    print('8')

    rclpy.init(args=args)
    print('9')
    arcturn_action_client = ArcTurnClient()
    print(10)
    arcturn_action_client.send_arc()

    try:
        rclpy.spin(arcturn_action_client)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
        rclpy.shutdown()
    finally:
        print("Done")


if __name__ == '__main__':
    main()
