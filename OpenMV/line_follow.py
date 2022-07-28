'''
line_follow.py
Tufts Create®3 Educational Robot Example
by Maddie Pero 

In this example we use an OpenMV camera to turn the robot into a line follower. It will follow a black
line on a white background and stop and wait at any blue intersections
'''

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

''' 
import necessary actions and messages
'''

from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

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


class LineFollow(Node):
    '''
    The LineFollow class is created which is a subclass of Node.
    This defines the class' constructor.
    '''

    def __init__(self):
        super().__init__('linefollow')
        
        '''
        Here we initialize the necessary publishers, subscribers, clients and classes 
        that will get called later on in the script. They will publish wheel velocities to the /cmd_vel topic on the robot, open the
        TCP server and edit parameters set by the motion control node
        '''

        self.tcp = TCPserver(CREATE_IP,CREATE_PORT)
        
        self.wheels_publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.wheels = Twist()
        self.linear = Vector3()
        self.angular = Vector3()
        
        self.client = self.create_client(SetParameters, namespace + '/motion_control/set_parameters')

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
        in order to get the robot to follow a black line we need it to follow on edge of the black tape. In this case
        we want it follow the right edge of the tape. to find different ranges of color we test the camera in its given lighting conditions.
        these if statements break down ranges of how the robot needs to move in order to follow the line
        '''
        if 100 <= r <= 120 and 100 <= g <= 120 and 100 <= b <= 120:
            ''''
            if the robot is in its sweet spot along the right edge of the black it needs
            to go straight
            ''' 
            self.send_speed(0.01,0)
            
        elif 100 >= r and 100 >= g and 100 >= b: 
            '''
            if the robot is too far left the r,g,b values will drop and the robot will need to move 
            to the right. Since it will be more difficult to make sharp right hand turns we will over correct the
            the robot whenever it sees too much black
            '''
            self.send_speed(0.01,-0.4)
            
        elif r>=120 and g>=120 and b>=120:
            '''
            if the robot is too far to the right all it will see is the white table and it will 
            need to move left
            '''
            self.send_speed(0.01,0.1)
            
        elif b>=100 and r<=100 and g<=100:
            '''
            when the robot sees blue intersections in the black lines we will have it stop. wait and 
            then move foward a set amount
            '''
            print('blue')
            self.send_speed(0,0)
            time.sleep(3)
            self.send_speed(0.2,0)
        
    def get_data(self):
        '''
        in this function we call two functions we wrote before to read in the data from the serial port and then preform color
        detection and send motions to the robot
        '''
        data = self.tcp.read() #can be parsed like a string 
        speed = self.color_detection(data)

def main(args=None):
    '''
    The rclpy library is initialized.
    '''
    rclpy.init(args=args)
    
    '''
    The node is created and can be used in other parts of the script.
    '''
    linefollow = LineFollow()


    '''
    we call the set_params() function to override the safety control paramter on the robot
    '''
    linefollow.set_params()
    
    '''
    we want the robot to continously check the serial port for data
    so we put the get_data() function in a while loop with a very short sleep time
    '''
    while True:
        linefollow.get_data()
        time.sleep(0.1)
    try:
        '''
        The node is "spun" so the functions can be executed. 
        '''
        rclpy.spin(linefollow)
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
