import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector


from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import DriveArc

counter = 0
print('counter is ' + str(counter))

class BumperTurn(Node):
    counter = 0
    def __init__(self):
        super().__init__('bumper_turn')
        self.subscription = self.create_subscription(
            HazardDetectionVector, 'Ygritte/hazard_detection', self.listener_callback, qos_profile_sensor_data)
        self._action_client = ActionClient(self, RotateAngle, 'Ygritte/rotate_angle')


    def listener_callback(self, msg):
        for detection in msg.detections:
            #print(str(msg.detections))
            message = str(msg.detections)
            #if 'base_link' in message:
             #   print('here')
        #for detection in range(1):
        # check here
            det = detection.header.frame_id
            #print('msg is ' + str(msg.detections))
            #print('type is '+ str(type(msg.detections)))
            #print(len(msg.detections))
            if det != "base_link":
                print(det)
                if det == "bump_right":
                    self.turn_goal(angle=0.25)
                    flag = False
                elif det == "bump_left":
                    self.turn_goal(angle=-0.25)
                    flag = False
                elif det == "bump_front_left":
                    self.turn_goal(angle=-1.3)
                    flag = False
                elif det == "bump_front_right":
                    self.turn_goal(angle=1.3)
                    flag = False
                elif det == "bump_front_center":
                    self.turn_goal(angle=1.57)
                    flag = False
            
        # arc def
                print('check 4')
        #rclpy.init(args=args)
                action_client = DriveArcActionClient()
                print('check 5')

                action_client.arc_goal()
                rclpy.spin(action_client)
            
            
            #if flag == False:
            #    break
                #flag = False
            #print('check 1')
                #rclpy.shutdown()
        #return flag

    def turn_goal(self, angle=1.57, max_rotation_speed=0.5):
        print('counter1 is ' + str(counter))
        #global counter
        #if counter <1:
        goal_msg = RotateAngle.Goal()
        print('check 1')
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed
        self._action_client.wait_for_server()
        print('check 2')
            #flag1 = False
#           counter = 1
        print('counter2 is ' + str(counter))
        #flag = False
        print('check 3')
        
        #time.sleep(5)
       
        
        return self._action_client.send_goal_async(goal_msg)
        
    flag = False
        

class DriveArcActionClient(Node):
    def __init__(self):
    
        super().__init__('drive_arc_action_client')
           
        self._action_client = ActionClient(
            self, DriveArc, 'Ygritte/drive_arc')

    def arc_goal(self, angle=3.14, radius=0.3, translate_direction=1, max_translation_speed=0.3):
        goal_msg = DriveArc.Goal()
        goal_msg.angle = angle
        goal_msg.max_translation_speed = max_translation_speed
        goal_msg.radius = radius
        goal_msg.translate_direction = translate_direction

        self._action_client.wait_for_server()
        

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):

        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

        rclpy.shutdown()

def turn(args=None):
    counter = 0
    rclpy.init(args=args)
    action_client = BumperTurn()
    action_client.turn_goal()
    flag = False
    rclpy.spin(action_client)
    flag = False
    rclpy.shutdown()
    print(str(flag))
    return flag

    
def arc(args=None):  

    rclpy.init(args=args)
    action_client = DriveArcActionClient()

    action_client.arc_goal()
    rclpy.spin(action_client)



def main(args=None):
    counter = 0
    flag = True
    turn()
    arc()
    #while flag:
    #	print(str(flag))
    #	turn()
    #	print('flag2 is ' + str(flag))
    #arc()


if __name__ == '__main__':
    main()
