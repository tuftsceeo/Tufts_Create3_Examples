'''
airtable_telerobot.py
Tufts Create®3 Educational Robot Example
by Maddie Pero 

In this example we will use airtable to remotely control the Create®3.
'''
import sys
import rclpy
from rclpy.node import Node

'''
These statements allow us to make get requests of the Airtable API & parse that information
'''
import requests
import json 

'''
These statements import iRobot Create®3 messages and actions.
'''
from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

'''
edit the namespace to match the namespace of your robot
'''
namespace = '[Namespace]'


class WheelVel(Node):
    '''
    The WheelVel class is created which is a subclass of Node.
    This defines the class' constructor.
    '''
    def __init__(self):    
        '''
        The following line calls the Node class's constructor and gives it the Node name,
        which is 'wheel_vel.'
        '''
        super().__init__('wheel_vel')

        '''
        Here we are declaring how we want the Node to publish a message. 
        We've imported Twist from geometry messages which is the type of message we will send over the topic '/cmd_vel' with a queue size of 10.
        Queue size is a quality of service setting that limiits amount of queued messages.
        Basically, we are determining what type of data we want to publish. 
        '''
        self.wheels_publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        
        '''
        The timer allows the callback to execute every 1 seconds, with a counter iniitialized.
        '''
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        '''
        define messag types to be used later 
        '''
        self.wheels = Twist()
        self.linear = Vector3()
        self.angular = Vector3()
        
    def timer_callback(self):
        ''' This function will be called every second with the timer. It makes a get request to the airtable API which will tell us how fast to spin the wheels'''
        
        API_KEY = ''
        BASE_ID = 'appMrPQ9Pz1FQDQ02'
        URL = "https://api.airtable.com/v0/" + BASE_ID + "/Create%20Telerobot?api_key=" + API_KEY
        
        
        r = requests.get(url = URL, params = {})
        '''
        The get request data comes in as a json package. We will convert this json package to a python dictionary so that it can be parsed
        '''
        data = r.json()
        return data
    	
    	

    def set_wheels(self, data):
        '''
        In this function we define the messages we want to publish and then publish them to the cmd_vel topic. We must define both angular and linear velocities to 
        send to the wheel of the robot. 
        '''
        
        
        self.linear.x = float(data['records'][0]['fields']['X'])
        self.linear.y = float(data['records'][0]['fields']['Y'])
        self.linear.z = float(data['records'][0]['fields']['Z'])
        
        self.angular.x = float(data['records'][1]['fields']['X'])
        self.angular.y = float(data['records'][1]['fields']['Y'])
        self.angular.z = float(data['records'][1]['fields']['Z'])
        
        self.wheels.linear = self.linear
        self.wheels.angular = self.angular
        self.wheels_publisher.publish(self.wheels)



def main(args=None):
    '''
    The rclpy library is initialized.
    '''
    rclpy.init(args=args)
    
    '''
    The node is created and can be used in other parts of the script.
    '''
    wheel_vel = WheelVel()
    
    '''
    We then can call the timer_callback() function to get the air table information. And call the set_wheels() function
    in a while loop to continually publish data to the topic.
    '''
    data = wheel_vel.timer_callback()
    while True:
    	wheel_vel.set_wheels(data)

    '''
    The node is "spun" so the callbacks can be called.
    '''
    try:
        rclpy.spin(wheel_vel)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
        rclpy.shutdown()
    finally:
        print("Done")  # Destroy the node explicitly
        


if __name__ == '__main__':
    main()
