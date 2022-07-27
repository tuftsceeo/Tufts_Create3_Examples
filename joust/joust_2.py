import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import DriveArc
from irobot_create_msgs.action import DriveDistance

namespace = 'Ygritte'
distance = 10
barrels = 4
count = int(barrels - 1)
max_rotation_speed=0.5
radius = (distance/75) + 0.16

'''
First action client class. 
'''
class ArcTurnClient(Node):

    def __init__(self):
        
        super().__init__('arcturn_action_client')
        self.drive = ActionClient(self, DriveDistance, namespace + '/drive_distance')
          
        self.turn = ActionClient(self, RotateAngle, namespace + '/rotate_angle')
        self.arc = ActionClient(self, DriveArc, namespace + '/drive_arc')  
        self.bump = self.create_subscription(
            HazardDetectionVector, namespace + '/hazard_detection', self.listener_callback, qos_profile_sensor_data)
        
        self.i = 0
        
    def listener_callback(self, msg):

        for detection in msg.detections:
            det = detection.header.frame_id
            if det != "base_link":
                print(det)
                if det == "bump_right":
                    self.send_turn(max_rotation_speed, angle=0.25)
                elif det == "bump_left":
                    self.send_turn(max_rotation_speed, angle=-0.25)
                elif det == "bump_front_left":
                    self.send_turn(max_rotation_speed, angle=-1.3)
                elif det == "bump_front_right":
                    self.send_turn(max_rotation_speed, angle=1.3)
                elif det == "bump_front_center":
                    self.send_turn(max_rotation_speed, angle=1.57)
            else:
                self.send_turn(max_rotation_speed, angle=0)


    def send_arc(self, angle=3.14, translate_direction=1, max_translation_speed=0.3):
        '''
        This function sends the arc goal (opposite directions for each iteration).
        '''
        if (int(self.i) % 2) == 0: 
            angle=3.14
        else:
            angle = -3.14
        translate_direction=1
        radius = (distance/75) + 0.16
        max_translation_speed=0.3
        goal_msg = DriveArc.Goal()
        goal_msg.angle = angle
        goal_msg.max_translation_speed = max_translation_speed
        goal_msg.radius = radius
        goal_msg.translate_direction = translate_direction

        self.arc.wait_for_server()
        
        print('Sending arc goal to server.')
        self._send_goal_future = self.arc.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.arc_response_callback)
        
        
    def send_turn(self, max_rotation_speed, angle):
        
        goal_msg_turn = RotateAngle.Goal()
        goal_msg_turn.angle = angle
        goal_msg_turn.max_rotation_speed = max_rotation_speed

        self.turn.wait_for_server()
        print('Sending turn goal to server.')
        self._send_goal_future = self.turn.send_goal_async(goal_msg_turn)
        self._send_goal_future.add_done_callback(self.turn_response_callback)
        time.sleep(2)
        
    def send_drive(self, distance=0.5, max_translation_speed=0.15):

        goal_msg = DriveDistance.Goal()
        goal_msg.distance = 0.3
        goal_msg.max_translation_speed = 1.0

        print('Waiting for action server to be available...')
        self.drive.wait_for_server()

        print('Sending drive goal to server.')
        self._send_goal_future = self.drive.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.drive_response_callback)

    def arc_response_callback(self, future):
        print('4')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Arc goal accepted :)')
        

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.arc_result_callback)
        
    def turn_response_callback(self, future):
        print('5')

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Turn goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.turn_result_callback)
        
    def drive_response_callback(self, future):

        print('Checking if goal was accepted or rejected...')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Drive goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.drive_result_callback)

    def arc_result_callback(self, future):
        print('arc result callback')
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        
        
    def turn_result_callback(self, future):
        print('turn result callback')

        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        #self.send_arc()
        if self.i < barrels:
            self.i += 1
            print('self.i is ' + str(self.i))
            self.send_arc()
        else:
            print('Shutting down action client node.')
            rclpy.shutdown()

    def drive_result_callback(self, future):

        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        
        

def main(args=None):
    print('main')

    rclpy.init(args=args)
    
    arcturn_action_client = ArcTurnClient()
    speed = 0.15
    arcturn_action_client.send_drive()
    print('9')
    time.sleep(4)
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
