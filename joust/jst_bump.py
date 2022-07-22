import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector


from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle


class BumperTurn(Node):
    def __init__(self):
        super().__init__('bumper_turn')
        self.subscription = self.create_subscription(
            HazardDetectionVector, 'Ygritte/hazard_detection', self.listener_callback, qos_profile_sensor_data)
        self._action_client = ActionClient(self, RotateAngle, 'Ygritte/rotate_angle')

    def listener_callback(self, msg):
      for detection in msg.detections:
        det = detection.header.frame_id
        if det != "base_link":
          print(det)
          if det == "bump_right":
            self.send_goal(angle=0.25)
          elif det == "bump_left":
            self.send_goal(angle=-0.25)
          elif det == "bump_front_left":
            self.send_goal(angle=-1.3)
          elif det == "bump_front_right":
            self.send_goal(angle=1.3)
          elif det == "bump_front_center":
            self.send_goal(angle=1.57)

    def send_goal(self, angle=1.57, max_rotation_speed=0.5):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def turn(args=None):
    angle = 1.57
    speed = 0.5   

    rclpy.init(args=args)
    action_client = BumperTurn()

    #action_client.send_goal(angle, speed)
    action_client.send_goal()
    rclpy.spin(action_client)
    time.sleep(0.5)

def main(args=None):
  turn()


if __name__ == '__main__':
    main()
