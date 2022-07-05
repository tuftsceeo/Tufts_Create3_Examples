import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import WheelStatus 

from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds

import inspect 

class ColorPalette(): 
    def __init__(self):
    	self.color1= LedColor(red= 153, green= 255, blue= 51)
    	self.color2= LedColor(red= 204, green= 0, blue= 255)
    	self.white= LedColor(red=255, green=255, blue=255)
    	
class Wheel_Subscriber(Node):

#want to create a node that subscribes to the wheel status

    def __init__(self, namespace: str = '/JonSnow'):
    	
#initialize by calling the node constructor and naming the node Wheel_Sub
#builds the subscriber, publisher and defines the audionote message
	
    	super().__init__('Wheel_Sub')
    	self.subscription = self.create_subscription(WheelStatus, namespace + '/wheel_status',self.listener_callback, qos_profile_sensor_data)
    	self.publisher = self.create_publisher(LightringLeds, namespace + '/cmd_lightring', 10)
    	
    	self.cp=ColorPalette()
    	self.lightring = LightringLeds()
    	self.lightring.override_system = True 
    	
    	
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
    		publish_color = [self.cp.color1, self.cp.color1, self.cp.color1, self.cp.color1, self.cp.color1, self.cp.color1]
    		
    	else :
    		publish_color = [self.cp.color2, self.cp.color2, self.cp.color2, self.cp.color2, self.cp.color2, self.cp.color2,]
    	
    	self.lightring.override_system = True 	
    	self.lightring.leds = publish_color
    	self.publisher.publish(self.lightring)
    	
    	
    def reset(self):
    	white = [self.cp.white, self.cp.white, self.cp.white, self.cp.white, self.cp.white, self.cp.white,]
    	self.lightring.leds=white
    	self.lightring.override_system = False
    	self.publisher.publish(self.lightring)
	
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
        Wheel_Sub.reset()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
