import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String

import socket
import time

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
        
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #self.uart_subscription = self.create_subscription(String, namespace + '/uart', self.listener_callback, qos_profile_sensor_data)

        self.uart_publisher = self.create_publisher(String,'/uart', 10)

        self.tcp = TCPserver(CREATE_IP,CREATE_PORT)


    #def listener_callback(self, msg:String):
        #self.tcp.send(extract string from message)

    def timer_callback(self):
        data = self.tcp.read() #can be parsed like a string 
        print(data)
        if len(data) != 0:
        	msg = String()
        	msg.data = data #but then need to put it in a message
        	self.uart_publisher.publish(msg)
        else: 
        	msg = String()
        	msg.data = 'nothing to publish'
        	self.uart_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    uartComm = TalkingTCP()
    try:
        rclpy.spin(uartComm)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    finally:
        print("Done")
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#1.connect to my Node
#2. Subscribe to my uart_rx
#3. Publish to my uart_tx
