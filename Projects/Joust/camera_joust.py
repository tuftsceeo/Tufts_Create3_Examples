'''
camera_joust.py
Tufts Create®3 Educational Robot Example
by Maddie Pero and Kate Wujciak

In this example we navigate the Create®3 robot through a serials of barrels before executing a joust over a wall. The start of the joust is
signaled by a physical flag detected by an OpenMV camera. 
'''

import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import DriveArc
from irobot_create_msgs.action import WallFollow
from builtin_interfaces.msg import Duration
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle

import socket

distance = 10

radius = (distance/75) + 0.16
translate_direction=1
max_translation_speed=0.3
namespace = '[Namespace]'
counter = 0

CREATE_IP = "10.245.91.235"
CREATE_PORT = 4004

class TCPserver():
    def __init__(self,IP,PORT):
        try:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error:
            print('Failed to create socket')
            sys.exit()

        print('Socket Created')
        
        '''
        Connect the socket object to the robot using IP address (string) and port (int)
        '''
        self.client.connect((IP,PORT))
        print('Socket Connected to ' + CREATE_IP)
	
    def read(self):
        print('read')
        '''
        Read the response sent by robot upon connecting. This message will be the serial data sent in by
        what is connected to the port. 
        '''
        try:
            msg = self.client.recv(1024).decode('ascii')
        except socket.error:
            print('Failed to read data')
        return msg

    def write(self,string):
        print('write')
        '''
        we can write serial data to the port as well using this function
        '''
        try:
            self.client.send(bytes(string.encode()))
        except socket.error:
            print('Failed to send data')

class DriveArcActionClient(Node):
    def __init__(self):
    
        super().__init__('drive_arc_action_client')
           
        self._action_client = ActionClient(
            self, DriveArc, namespace + '/drive_arc')
        self.drive = ActionClient(self, DriveDistance, namespace + '/drive_distance')
        
        self.tcp = TCPserver(CREATE_IP,CREATE_PORT)
        
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0 
        
    def timer_callback(self):
        data = self.tcp.read()
        r1 = data.find('(')
        r2 = data.find(')')
        newdata = data[r1+1:r2].split(', ')
        if newdata[0].isdigit() and newdata[1].isdigit() and newdata[2].isdigit(): 
            r = int(newdata[0])
            g = int(newdata[1])
            b = int(newdata[2])
        else: 
            r = 0
            g = 0 
            b = 0 
        print(r,g,b)
        if r <= 50 and g<= 50 and b <= 50:
            print('black')
            go = 'no'
        else: 
            go = 'yes'
        return go
        
    def send_goal(self, angle, radius, translate_direction, max_translation_speed):
        goal_msg = DriveArc.Goal()
        goal_msg.angle = angle
        goal_msg.max_translation_speed = max_translation_speed
        goal_msg.radius = radius
        goal_msg.translate_direction = translate_direction

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        
    def send_drive(self, distance=0.5, max_translation_speed=0.15):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = 0.3
        goal_msg.max_translation_speed = 1.0

        print('Waiting for action server to be available...')
        self.drive.wait_for_server()

        print('Sending drive goal to server.')
        self._send_goal_future = self.drive.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.drive_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def drive_response_callback(self, future):
        print('Checking if goal was accepted or rejected...')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Drive goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.drive_result_callback)        


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        print('shutting down node')
        rclpy.shutdown()

    def drive_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

        
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
     
 
    def send_goal(self, follow_side=-1, max_runtime=10):
        print('Ready for Action')
         
        goal_msg = WallFollow.Goal()
        goal_msg.follow_side = follow_side
            
        goal_msg.max_runtime = Duration(sec=10, nanosec=0)
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
        
class RotateActionClient(Node):
    '''
    This is an action client. Action clients send goal requests to action servers,
    which sends goal feedback and results back to action clients. This action client
    tells the robot to turn 90 degrees at a speed of 0.15. Subclass of Node.
    '''

    def __init__(self):

        print('Initializing a new action server in order to rotate.')
        super().__init__('rotate_action_client')

                
        self._action_client = ActionClient(self, RotateAngle, namespace + '/rotate_angle')

    def send_goal(self, angle=1.57, max_rotation_speed=0.5):

        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle 
        goal_msg.max_rotation_speed = max_rotation_speed

        print('Waiting for action server to be available...')
        self._action_client.wait_for_server()
 
        print('Action server available. Sending rotate goal to server.')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
       
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        print('Checking if goal was accepted or rejected...')
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
     
        print('Shutting down rotate action client node.')
        rclpy.shutdown()

def arc1(args=None):
    global counter
    angle = 3.14

    rclpy.init(args=args)
    action_client = DriveArcActionClient()
    speed = 0.15
    green_flag = action_client.timer_callback()
    while green_flag == 'no':
       green_flag = action_client.timer_callback()
    if (counter % 2) == 0: 
        angle=3.14        
    else:
        angle = -3.14
        
    if counter == 0:
        action_client.send_drive()
    counter += 1
    time.sleep(2)         
    action_client.send_goal(angle, radius, translate_direction, max_translation_speed)
    rclpy.spin(action_client)

def turn(args=None):
    angle = 0.3
    speed = 0.5   
    
    rclpy.init(args=args)
    action_client = RotateActionClient()
    
    action_client.send_goal(angle, speed)
    rclpy.spin(action_client)
    time.sleep(0.5) 
    
def wall(args=None):
    rclpy.init(args=args)
     
    wall_client = WallFollowActionClient()
    wall_client.send_goal(follow_side=-1, max_runtime=10)
    rclpy.spin(wall_client)


def main(args=None):
    for i in range(2):
        arc1()
        time.sleep(2)
    turn()
    wall()


if __name__ == '__main__':
    main()
