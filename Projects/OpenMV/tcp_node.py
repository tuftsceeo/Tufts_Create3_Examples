'''
tcp_node.py
Tufts Create®3 Educational Robot Example
by Maddie Pero 

This script is the base file for running a tcp server to communicate via the serial port on the Create®3.
'''

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

'''
import neccessary messages and modules
'''
from std_msgs.msg import String

import socket
import time

CREATE_IP = "0.0.0.0" '''put in your robot's IP address'''
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


class TalkingTCP(Node):
    '''
    The TalkingTCP class is created which is a subclass of Node.
    This defines the class' constructor.
    '''
    def __init__(self):
        super().__init__('uartComm')
        
	'''
	Here we initialize a timer, a publisher that will publish a string to the /uart topic, and the prior tcp class for easier use
	'''
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.uart_publisher = self.create_publisher(String,'/uart', 10)

        self.tcp = TCPserver(CREATE_IP,CREATE_PORT)

    def timer_callback(self):
	'''
	Once every second this function will be called. It reads in the serial data from the Create®3. 
	Formats is properly and then publishes it to the /uart topic. If there is no data to publish, 
	we will send the message 'nothing to publish'
	'''
        data = self.tcp.read()  
        print(data)
        if len(data) != 0:
        	msg = String()
        	msg.data = data 
        	self.uart_publisher.publish(msg)
        else: 
        	msg = String()
        	msg.data = 'nothing to publish'
        	self.uart_publisher.publish(msg)


def main(args=None):
	
    '''
    The rclpy library is initialized.
    '''
    rclpy.init(args=args)
	
    '''
    The node is created and can be used in other parts of the script.
    '''
    uartComm = TalkingTCP()
    try:
        '''
        The node is "spun" so the functions can be executed. 
        '''
        rclpy.spin(uartComm)
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

