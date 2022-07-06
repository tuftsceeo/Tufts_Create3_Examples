#Publish to a Topic Example
#Tufts Create3 Examples
#Maddie Pero 

#In this example we will publish random colors to the led ring on the Create3 

#import global modules
import sys
import rclpy
from rclpy.node import Node
import random

#import iRobot Create3 messages
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds



class ColorPalette():      #define a class of frequently used colors for easy access 
    def __init__(self):
        self.red = LedColor(red=255, green=0, blue=0)
        self.green = LedColor(red=0, green=255, blue=0)
        self.blue = LedColor(red=0, green=0, blue=255)
        self.yellow = LedColor(red=255, green=255, blue=0)
        self.pink = LedColor(red=255, green=0, blue=255)
        self.cyan = LedColor(red=0, green=255, blue=255)
        self.purple = LedColor(red=127, green=0, blue=255)
        self.white = LedColor(red=255, green=255, blue=255)
        self.grey = LedColor(red=189, green=189, blue=189)
        self.tufts_blue = LedColor(red=98, green=166, blue=10)
        self.tufts_brown = LedColor(red=94, green=75, blue=60)

class LEDPublisher(Node):
    def __init__(self, namespace: str = "[Namespace]"):     
        super().__init__('led_publisher')

        self.cp = ColorPalette()
        self.lights_publisher = self.create_publisher(        #initialize the publisher 
            LightringLeds, namespace + '/cmd_lightring', 10)

        timer_period = 2 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) #create a timer 
        self.lightring = LightringLeds()
        self.lightring.override_system = True

    def timer_callback(self):                     #this function will get called every 2 seconds 
        all_colors = [self.cp.white, self.cp.red, self.cp.green, self.cp.blue, self.cp.yellow, self.cp.pink, self.cp.cyan, self.cp.purple, self.cp.grey] #define all the color choices
        
        led_colors = [random.choice(all_colors), random.choice(all_colors),random.choice(all_colors),random.choice(all_colors), random.choice(all_colors), random.choice(all_colors)] #randomly choose colors to publish 
        
        current_time = self.get_clock().now()

        self.lightring.leds = led_colors 
        self.lightring.header.stamp = current_time.to_msg()
        self.lights_publisher.publish(self.lightring)

    def reset(self):  #release control of the lights back to the robot
        self.lightring.override_system = False
        white = [self.cp.white, self.cp.white, self.cp.white,
                 self.cp.white, self.cp.white, self.cp.white]
        self.lightring.leds = white

        self.lights_publisher.publish(self.lightring)


def main(args=None):
    rclpy.init(args=args)

    led_publisher = LEDPublisher()

    try:
        rclpy.spin(led_publisher)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    finally:
        print("Done")  # Destroy the node explicitly
        led_publisher.reset()
        led_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
