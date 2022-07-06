import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
#from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import IrIntensityVector
# from sensor_msgs.msg import BatteryState


class IRSubscriber(Node):
    '''
    An example of subscribing to a ROS2 topic.
    A Node listening to the /ir_intensity topic.
    '''
    def __init__(self, namespace: str = "/JonSnow"):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        'IR_subscriber'
        '''
        super().__init__('IR_subscriber')
        self.subscription = self.create_subscription(
            IrIntensityVector, namespace + '/ir_intensity', self.listener_callback,
            qos_profile_sensor_data)

    def listener_callback(self, msg:IrIntensityVector):
        '''
        Purpose
        -------
        Whenever our subscriber (listener) gets a message this function is 
        'called back' to and ran.
        '''
        #self.get_logger().info('I heard: "%s"' % msg)
        self.printIR(msg)

    def printIR(self, msg):
        '''
        :type msg: IrIntensity
        :rtype: None
        An example of how to get components of the msg returned from a topic.
        '''
        # We can get components of the message by using the '.' dot operator
        for reading in msg.readings: 
        	val = reading.value
        	print("IR Sensor:", val)


def main(args=None):
    rclpy.init(args=args)

    IR_subscriber = IRSubscriber()
    try:
        rclpy.spin(IR_subscriber)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        IR_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
