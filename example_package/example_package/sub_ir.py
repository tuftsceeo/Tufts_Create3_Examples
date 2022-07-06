import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
#from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import IrIntensity
# from sensor_msgs.msg import BatteryState

print('check 1')

class IRSubscriber(Node):
    '''
    An example of subscribing to a ROS2 topic.
    A Node listening to the /ir_intensity topic.
    '''
    print('check 2')
    def __init__(self, namespace: str = "/Ygritte"):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        'IR_subscriber'
        '''
        super().__init__('IR_subscriber')
        self.subscription = self.create_subscription(
            IrIntensity, namespace + '/ir_intensity', self.listener_callback,
            qos_profile_sensor_data)
        print('check 3')

    def listener_callback(self, msg: IrIntensity):
        '''
        Purpose
        -------
        Whenever our subscriber (listener) gets a message this function is 
        'called back' to and ran.
        '''
        print('mid check')
        self.get_logger().info('I heard: "%s"' % msg)
        self.printIR(msg)
        print('check 4')

    def printIR(self, msg):
        print('check 5')
        '''
        :type msg: IrIntensity
        :rtype: None

        An example of how to get components of the msg returned from a topic.
        '''
        # We can get components of the message by using the '.' dot operator
        print("IR Sensor:", msg)
        print("IR Sensor:", msg.percentage)


def main(args=None):
    print('check 6')
    rclpy.init(args=args)
    print('check 7')

    IR_subscriber = IRSubscriber()
    print('check 8')
    try:
        print('check 9')
        rclpy.spin(IR_subscriber)
        print('check 10')
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        IR_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()