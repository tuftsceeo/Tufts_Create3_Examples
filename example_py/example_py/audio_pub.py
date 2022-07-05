import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import WheelStatus 
from irobot_create_msgs.msg import AudioNote

from builtin_interfaces.msg import Duration 

import inspect 

class Wheel_Subscriber(Node): #want to create a node that subscribes to the wheel status
    def __init__(self, namespace: str = '/JonSnow'):
    #initialize by calling the node constructor and naming the node Wheel_Sub
    #builds the subscriber, publisher and defines the audionote message
    	super().__init__('Wheel_Sub')
    	self.audio = AudioNote()
    	self.subscription = self.create_subscription(WheelStatus, namespace + '/wheel_status',self.listener_callback, qos_profile_sensor_data)
    	self.publisher = self.create_publisher(AudioNote, namespace + '/cmd_audio', 10)
    	
    def listener_callback(self, msg: WheelStatus): 
#when we get a message this function is called back and ran
	
    	#self.get_logger().info('I heard: "%s"' % msg) 
    	self.printWheel(msg, msg.wheels_enabled)
	
    def printWheel(self, msg, wheels_enabled: bool):
    	
#defines printWheel function that gets called whenever listener_callback is called
#this function will print the wheel status
#if the wheels are enabled it will play a lower freq note
#if the wheels are disabled it will play a higher freq note
    	print("Wheels Are Enabled:", msg.wheels_enabled)
    	if wheels_enabled == True : 
    		self.audio.frequency = 440 
    		self.audio.max_runtime = Duration(sec = 1, nanosec = 0)
    		self.publisher.publish(self.audio)
    		
    	else :
    		self.audio.frequency = 880
    		self.audio.max_runtime = Duration(sec = 1, nanosec = 0) 
    		self.publisher.publish(self.audio)
    		
    	#(frequency=0, max_runtime=builtin_interfaces.msg.Duration(sec=0, nanosec=0))
	
def main(args=None):
    rclpy.init(args=args)	
    Wheel_Sub = Wheel_Subscriber()
    
    try:
        rclpy.spin(Wheel_Sub)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        Wheel_Sub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
