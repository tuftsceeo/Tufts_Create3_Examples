import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient

from std_msgs.msg import String

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


namespace = '/NightKing'
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

        #Connect the socket object to the robot using IP address (string) and port (int)
        self.client.connect((IP,PORT))
        print('Socket Connected to ' + CREATE_IP)
	
    def read(self):
        #Read the response sent by robot upon connecting
        try:
            msg = self.client.recv(1024).decode('ascii')
        except socket.error:
            print('Failed to read data')
        return msg

    def write(self,string):
        try:
            self.client.send(bytes(string.encode()))
        except socket.error:
            print('Failed to send data')

    def close(self):
        self.client.close()


class TalkingTCP(Node):

    def __init__(self):
        super().__init__('uartComm')
        
        #timer_period = 2 # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

        #self.uart_subscription = self.create_subscription(String, namespace + '/uart', self.listener_callback, qos_profile_sensor_data)

        self.uart_publisher = self.create_publisher(String,'/uart', 10)

        self.tcp = TCPserver(CREATE_IP,CREATE_PORT)
        
        self.wheels_publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.wheels = Twist()
        self.linear = Vector3()
        self.angular = Vector3()
        
        self.turn = ActionClient(self, RotateAngle, namespace + '/rotate_angle')
        
        self.client = self.create_client(SetParameters, namespace + '/motion_control/set_parameters')
        
    def send_turn(self,angle):
        goal_msg_turn = RotateAngle.Goal()
        goal_msg_turn.angle = angle
        goal_msg_turn.max_rotation_speed = 1.0
        
        self.turn.wait_for_server()
        self._send_goal_future = self.turn.send_goal_async(goal_msg_turn)

    def set_params(self):
        request = SetParameters.Request()
        param = Parameter() 
        param.name = "safety_override"
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = 'full'
        request.parameters.append(param)
        
        self.client.wait_for_service()
        self.future = self.client.call_async(request)
        
    def send_speed(self, linear_speed, angular_speed):
        self.linear.x = float(linear_speed)
        self.linear.y = float(linear_speed)
        self.linear.z = float(linear_speed)
        
        self.angular.x = float(angular_speed)
        self.angular.y = float(angular_speed)
        self.angular.z = float(angular_speed)
        
        self.wheels.linear = self.linear
        self.wheels.angular = self.angular
        self.wheels_publisher.publish(self.wheels)
        
    def color_detection(self,data):
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
        
        if 100 <= r <= 120 and 100 <= g <= 120 and 100 <= b <= 120: #if it is along the right side of the black line (sweet spot)
            self.send_speed(0.01,0)
        elif 100 >= r and 100 >= g and 100 >= b: #too far left (too black) 
            self.send_speed(0.01,-0.4)
        elif r>=120 and g>=120 and b>=120: #too far right needs (too white)
            self.send_speed(0.01,0.1)
        elif b>=100 and r<=100 and g<=100:
            print('blue')
            self.send_speed(0,0)
            time.sleep(3)
            self.send_speed(0.3,0)
        
    def get_data(self):
        data = self.tcp.read() #can be parsed like a string 
        speed = self.color_detection(data)

def main(args=None):
    rclpy.init(args=args)
    uartComm = TalkingTCP()
    uartComm.set_params()
    while True:
        uartComm.get_data()
        time.sleep(0.1)
    try:
        rclpy.spin(uartComm)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    finally:
        print("Done")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
