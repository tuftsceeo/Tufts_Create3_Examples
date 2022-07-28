'''
road_signs.py
Tufts Create®3 Educational Robot Example
by Maddie Pero 

In this example we use an OpenMV camera to recognize road signs and direct the motion of the robot. 
'''

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient

''' 
import necessary actions and messages
'''

from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from irobot_create_msgs.action import RotateAngle

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue 
from rcl_interfaces.msg import Parameter

import socket
import time

'''
create global variables that can be easily changes for each robot
'''

namespace = '/[Namespace]'
CREATE_IP = "0.0.0.0"
CREATE_PORT = 4004

class TCPserver():
    '''
    this class creates the TCP server that will read serial information in from a port on the robot
    '''
    def __init__(self,IP,PORT):
        '''
        first we need to create a socket that the robot can connect to
        '''
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
        '''
        we can write serial data to the port as well using this function
        '''
        try:
            self.client.send(bytes(string.encode()))
        except socket.error:
            print('Failed to send data')

    def close(self):
        '''
        this function will close the socket when we are done with it
        '''
        self.client.close()


class Road_Signs(Node):
    '''
    The Road_Signs class is created which is a subclass of Node.
    This defines the class' constructor.
    '''

    def __init__(self):
        super().__init__('road_signs')
        '''
        Here we initialize the necessary publishers, subscribers, clients and classes 
        that will get called later on in the script. They will publish wheel velocities to the /cmd_vel topic on the robot. 
	Send turn actions to the robot. Open the
        TCP server and edit parameters set by the motion control node
        '''
	
        self.tcp = TCPserver(CREATE_IP,CREATE_PORT)
        
        self.wheels_publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.wheels = Twist()
        self.linear = Vector3()
        self.angular = Vector3()
        
        self.turn = ActionClient(self, RotateAngle, namespace + '/rotate_angle')
        
        self.client = self.create_client(SetParameters, namespace + '/motion_control/set_parameters')
        
    def send_turn(self,angle):
        '''
        Define the action message that will get sent to the robot when we want it to turn
        '''
        goal_msg_turn = RotateAngle.Goal()
        goal_msg_turn.angle = angle
        goal_msg_turn.max_rotation_speed = 1.0
        
        '''
        wait for the server to be available and then send that action message asynchronously
        '''
        self.turn.wait_for_server()
        self._send_goal_future = self.turn.send_goal_async(goal_msg_turn)

    def set_params(self):
        '''
        This function will edit the safety override parameter of the robot
        and allow it to move in whatever direciton we want
        '''
        
        '''
        define the parameter that we wish to append. In this case we want the safety override parameter to be 'full'
        '''
        request = SetParameters.Request()
        param = Parameter() 
        param.name = "safety_override"
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = 'full'
        request.parameters.append(param)
        
        '''
        wait for a service client to be available then send that appended parameter
        '''
        self.client.wait_for_service()
        self.future = self.client.call_async(request)
        
    def send_speed(self, linear_speed, angular_speed):
        '''
        This function defines the wheel velocity message that will be published to the /cmd_vel topic
        and then publishes that message to the topic. In order for the robot to move it needs both a
        linear and angular velocity for the wheels
        '''
        self.linear.x = float(linear_speed)
        self.linear.y = float(linear_speed)
        self.linear.z = float(linear_speed)
        
        self.angular.x = float(angular_speed)
        self.angular.y = float(angular_speed)
        self.angular.z = float(angular_speed)
        
        self.wheels.linear = self.linear
        self.wheels.angular = self.angular
        
        '''
        publishes the message to the topic
        '''
        self.wheels_publisher.publish(self.wheels)
        
    def color_detection(self,data):
        '''
        this function will take in the serial data read by the robot, parse through it and determine what movement the 
        robot should preform based on the data
        '''
        '''
        the serial data comes in as one long string and needs to be parsed to get just the r,g,b integer values 
        that are the color readings from the camera. Sometimes the serial data does not come in perfectly
        formatted which is why there is an else statement that sets all values of r,g,b to 0 if the messsage
        cannot be parsed properly
        '''
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
        '''
        these if statements seperate teh different road sign colors the robot might encounter. they are red, yellow, green and none.
        in order to find the correct r,g,b range of each color we had to preform tests with the camera under different lighting. If the
        environment of the camera changes these values will also likely change. 
        '''
        if r >= 135 and g <= 110 and b <= 110:
            '''
            if the robot runs into a red road sign it will stop and then turn left 90 degrees
            '''
            print('red')
            self.send_speed(0,0)
            self.send_turn(1.57)
            time.sleep(0.1)
	
        elif g>= 135 and r<= 110 and b<= 110 :
            '''
            if the robot runs into a green road sign it will stop and turn around 180 degrees
            '''
            print('green')
            self.send_speed(0,0)
            self.send_turn(3.14)
            time.sleep(0.1)
	
        elif b<=55 and r>=130 and g>= 130:
            '''
            if the robot runs into a yellow road sign it will stop and turn 90 degrees to the right
            '''
            print('yellow')
            self.send_speed(0,0)
            self.send_turn(-1.57)
            time.sleep(0.1)
	
        else: 
           '''
           when the camera sees none of these colors the robot will drive straight down the road
           '''
           self.send_speed(0.1,0)
        
    def get_data(self):
        '''
        in this function we call two functions we wrote before to read in the data from the serial port and then preform color
        detection and send motions to the robot
        '''
        data = self.tcp.read() 
        speed = self.color_detection(data)

def main(args=None):
    '''
    The rclpy library is initialized.
    '''
    rclpy.init(args=args)
    
    '''
    The node is created and can be used in other parts of the script.
    '''
    road_signs = Road_Signs()
    
    '''
    we call the set_params() function to override the safety control paramter on the robot
    '''
    road_signs.set_params()
    
    '''
    then we want the robot to continously check the serial port for data
    so we put the get_data() function in a while loop with a very short sleep time
    '''
    while True:
        stop_signs.get_data()
        time.sleep(0.1)
    try:
        '''
        The node is "spun" so the callbacks can be called.
        '''
        rclpy.spin(road_signs)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    finally:
        '''
        finally the node is shut down when we are done running the script
        '''
        print("Done")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
