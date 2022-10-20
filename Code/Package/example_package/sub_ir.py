'''
sub_ir.py
Tufts Create®3 Educational Robot Example
by Kate Wujciak 

This file shows how to subscribe to a topic in ROS2 using the Create®3. It subscribes
to the IR sensor and displays the relevant information in your terminal. 

Today we are going to show you how to write a code that lets us subscribe to the 7 IR sensors 
on the robot. IR stands for infrared. These sensors are used to detect objects. It could help 
possibly avoid obstacles or know how far it is from a wall.
'''

'''
We're going to write statements that import messages, actions, interfaces, and variable types.
Import the rclpy module to gain access to the ros python client library
The rclpy module is our holy grail for this course.
'''

import sys
import rclpy

'''
We're going to be using sensor data from the IR vectors so we need to import
'''
from rclpy.node import Node
'''
from rclpy.qos (qos stands for quality of service) so that the node knows what type of info we will be using 
'''
from rclpy.qos import qos_profile_sensor_data
'''
and then... we find this by going to the irobot create messages github 
you can find other sensor messages such as battery state that are standard ros messages
like battery BatteryState for example

'''
from irobot_create_msgs.msg import IrIntensityVector

'''
Input your namespace here as a global variable. Helps if you are using multiple robots.
'''
namespace = '[Namespace]'

class IRSubscriber(Node):
    '''
    The IRSubscriber class is created which is a subclass of Node.
    The Node is subscribing to the /ir_intensity topic. Classes help us organize code so 
    we can bundle it together based on functionality.
    '''
    
    def __init__(self):
        '''
        The following line calls the Node class' constructor and declares a node name,
        which is 'IR_subscriber' in this case. 
        '''
        super().__init__('IR_subscriber')
        '''
        This following line indicates that the node is subscribing to the IrIntensityVector
        type over the '/ir_intensity' topic. Now we are creating a subscription to this topic.
        '''
        print('Creating subscription to to the IrIntensityVector type over the /ir_intensity topic')
        '''
        In these parentheses we have our IrIntensityVector type, the namespace and the topic its subscribing too, 
        when we want to subscribe to the message so on a listener callback which we will go over later, 
        and the type of quality of service data it’s receiving.
        '''
        self.subscription = self.create_subscription(
            IrIntensityVector, namespace + '/ir_intensity', self.listener_callback,
            qos_profile_sensor_data)
  
    def printIR(self, msg):
        '''
        Its purpose is to determine 
        which parts of the info are worth showing.
        We want to print the IR sensor values that are important to us. 
        We know what the IrIntensityVector msg spits out based on what irobot tells us
        in their github as mentioned earlier. 
        
        :type msg: IrIntensity
        :rtype: None
        The msg is returned from our topic '/ir_intensity.'
        To get components of a message, use the '.' dot operator. 
        '''
        print('Printing IR sensor readings:')
        
        '''
        So we're gonna loop through every reading in our list of readings from the msg.
        There is a header and a value within each reading
        So we'll get the value of that reading again by using the dot operator and print it 

        '''
        for reading in msg.readings: 
        	val = reading.value
        	print("IR Sensor: " + str(val))      
        
    def listener_callback(self, msg:IrIntensityVector):
        '''
        This is the function that runs everytime it gets a msg from the robot. 
        Earlier, in the initialized function, we told it 
        The subscriber's callback listens and as soon as it receives the message,
        this function runs. 
        This callback function is basically printing what it hears. It runs the data
        it receives in your terminal (msg).  
        '''
        print('Now listening to IR sensor readings it hears...')

        self.printIR(msg)


def main(args=None):
    '''
    This line initializes the rclpy library. 
    '''
    rclpy.init(args=args)
    '''
    This creates the node.
    '''
    IR_subscriber = IRSubscriber()
    '''
    Now we'll create the node we had earlier defined with our IRSubscriber(). 
    The node is then "spun" (start subscribing to the topic) so its callbacks are called.
    '''
    print('Callbacks are called.')
    try:
        rclpy.spin(IR_subscriber)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
    	'''
    	Destroying the node acts as a "reset" so we don't run into 
    	any problems when we try and run again.
    	'''
    	print("Done")
    	IR_subscriber.destroy_node()
    	rclpy.shutdown()


if __name__ == '__main__':
    main()
