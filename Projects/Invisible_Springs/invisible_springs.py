'''
invisbile_springs.py
Tufts Create®3 Educational Robot Example
by Maddie Pero 

This file is an example of a proportional controller. It will keep a pre-set distance from any object moving in front of it. 
To enforce a larger distance decrease both the max_speed and the threshold. 
'''

import sys
import rclpy

'''
Statements that import messages, actions, interfaces, and variable types.
'''
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.msg import IrIntensity

from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

'''
define global variables
'''

namespace = '[Namespace]'

max_speed = 4
threshold = 1000


class Springs(Node):
    '''
    The Springs class is created which is a subclass of Node.
    The Node will subscribe to the values of the IR sensors and publish a speed to the wheels so make them move. 
    '''
    
    def __init__(self):
        '''
        The following line calls the Node class' constructor and declares a node name,
        which is 'springs' in this case. 
        '''
        super().__init__('springs')
        '''
        Then we initiliaze the subscriber, publisher and message types that will be sent in them. 
        '''
        self.subscription = self.create_subscription(
            IrIntensityVector, namespace + '/ir_intensity', self.listener_callback,
            qos_profile_sensor_data)
        self.ir = IrIntensity()
        
        self.wheels_publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.wheels = Twist()
        self.linear = Vector3()
        self.angular = Vector3()

    def listener_callback(self, msg:IrIntensityVector):
        '''
        The subscriber's callback listens and as soon as it receives a message runs this listener_callback() function
        This callback function then calls another function ( SPEED() ). 
        '''
        self.SPEED(msg)

    def SPEED(self, msg):
        ''' 
        First we parse the message recieved from the IR sensors. It is a string made up of messages of an iRobot class IrIntensity.
        We create a new list of just the IR sensor values.
        '''
        val = []
        message = msg.readings
        for i in range(len(message)):
            val.append(message[i].value)
        #error = sum(val)/len(val)
        
        '''
        Then we use the max value in our list of IR values to determine the speed of the robot
        '''
        if 0 < max(val) < threshold:
        	speed = max_speed/max(val)
        elif max(val) >= threshold:
        	speed = 0 
        else: 
        	speed = max_speed 
        
        print(speed)
        
        '''
        finally we define the messages needed to move the robot and publish them to the /cmd_vel topic
        '''
        self.linear.x = float(speed)
        self.linear.y = float(speed)
        self.linear.z = float(speed)
        
        self.angular.x = float(0.0)
        self.angular.y = float(0.0)
        self.angular.z = float(0.0)
        
        self.wheels.linear = self.linear
        self.wheels.angular = self.angular
        self.wheels_publisher.publish(self.wheels)

def main(args=None):
    '''
    This line initializes the rclpy library. 
    '''
    rclpy.init(args=args)
    '''
    This creates the node.
    '''
    springs = Springs()
    '''
    The node is then "spun" so its callbacks are called.
    '''
    try:
        rclpy.spin(springs)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
        '''
        when we want to stop the program we must shut down the node
        '''
        rclpy.shutdown()
    finally:
    	print("Done")
    	
if __name__ == '__main__':
    main()
