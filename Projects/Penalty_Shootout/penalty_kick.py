'''
penalty_kick.py
Tufts Create®3 example
by Maddie Pero

This is an example of how to get your Create®3 robot to preform a Paneka. 
'''

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

'''
import messages/actions
'''
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import DriveArc
from irobot_create_msgs.action import NavigateToPosition

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue 
from rcl_interfaces.msg import Parameter

namespace = '[Namespace]'

class PenaltyKick1(Node):

    def __init__(self):
    	super().__init__('penalty_kick_1')
    	'''
    	Here we initialize each of the action clients we are going to use & the interface service client.
    	'''
    	self.client = self.create_client(SetParameters, namespace + '/motion_control/set_parameters')
    	self.drive = ActionClient(self, DriveDistance, namespace + '/drive_distance')
    	self.turn = ActionClient(self, RotateAngle, namespace + '/rotate_angle')
    	self.arc = ActionClient(self, DriveArc, namespace + '/drive_arc')
    	self.pos = ActionClient(self, NavigateToPosition, namespace + '/navigate_to_position')
    	
    def set_params(self):
        '''
        Override the safety control paramter in another node so that we can drive backwards.
        '''
        
        '''
        get the request we will send to the service server
        '''
        request = SetParameters.Request()
        
        '''
        edit that paramter message with the correct parameter name & value
        '''
        param = Parameter() 
        param.name = "safety_override"
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = 'full'
        '''
        append the service request with the correct parameter message
        '''
        request.parameters.append(param)
        
        '''
        wait until the service server is available then send the request
        '''
        self.client.wait_for_service()
        self.future = self.client.call_async(request)
    
    def move1(self):
        '''
        Define the goal message we want to send to the robot. In this case its a given drive distance and speed.
        '''
        goal_msg1 = DriveDistance.Goal()
        goal_msg1.distance = -0.25
        goal_msg1.max_translation_speed = 2.0
        
        '''
        wait to send the message once the action server is available. send that message asynchronously and then add a callback 
        so that we know when the future is completed
        '''
        self.drive.wait_for_server()
        self._send_goal_future = self.drive.send_goal_async(goal_msg1)
        self._send_goal_future.add_done_callback(self.move1_response_callback)
        
    def move1_response_callback(self, future):
    	'''
    	find out if the server accepted the goal & log that information
    	'''
    	goal_handle1 = future.result()
    	if not goal_handle1.accepted:
    		self.get_logger().info('Goal rejected :(')
    		return
    		
    	self.get_logger().info('Goal accepted :)')
    	
    	'''
    	get the result of the action & register a future callback to know when that result is ready
    	'''
    	self._get_result_future = goal_handle1.get_result_async()
    	self._get_result_future.add_done_callback(self.move1_result_callback)

    def move1_result_callback(self, future):
    	'''
    	log the result of the action and move on to the next move
    	'''
    	result = future.result().result
    	self.get_logger().info('Result: {0}'.format(result))
    	print('now move2')
    	self.move2()
        
        
    ''' 
    Repeat the same steps as in move1 with different action clients to design your penalty kick approach.
    '''
    def move2(self):
        goal_msg2 = DriveArc.Goal()
        goal_msg2.translate_direction = 1
        goal_msg2.angle = .57
        goal_msg2.radius = .4
        goal_msg2.max_translation_speed = 2.0
        
        self.arc.wait_for_server()
        self._send_goal_future = self.arc.send_goal_async(goal_msg2)
        self._send_goal_future.add_done_callback(self.move2_response_callback)
        
    def move2_response_callback(self, future):
    	goal_handle2 = future.result()
    	if not goal_handle2.accepted:
    		self.get_logger().info('Goal rejected :(')
    		return
    	self.get_logger().info('Goal accepted :)')
    	self._get_result_future = goal_handle2.get_result_async()
    	self._get_result_future.add_done_callback(self.move2_result_callback)

    def move2_result_callback(self, future):
    	result = future.result().result
    	self.get_logger().info('Result: {0}'.format(result))
    	print('now move3')
    	self.move3()
        
    def move3(self):
        goal_msg3 = RotateAngle.Goal()
        goal_msg3.max_rotation_speed = 2.0
        goal_msg3.angle = -1.0
        
        self.turn.wait_for_server()
        self._send_goal_future = self.turn.send_goal_async(goal_msg3)
        self._send_goal_future.add_done_callback(self.move3_response_callback)
        
    def move3_response_callback(self, future):
    	goal_handle3 = future.result()
    	if not goal_handle3.accepted:
    		self.get_logger().info('Goal rejected :(')
    		return
    	self.get_logger().info('Goal accepted :)')
    	self._get_result_future = goal_handle3.get_result_async()
    	self._get_result_future.add_done_callback(self.move3_result_callback)

    def move3_result_callback(self, future):
    	result = future.result().result
    	self.get_logger().info('Result: {0}'.format(result))
    	
    	'''
    	once all 3 moves have been completed shutdown the node
    	'''
    	print('SHOOT')
    	rclpy.shutdown()
        
def main(args=None):
    '''
    initialize rclpy and the node we created
    '''
    rclpy.init(args=args)
    penalty_kick_1 = PenaltyKick1() 
    
    '''
    call the functions to set the paramters & start the move sequence
    '''
    penalty_kick_1.set_params()
    penalty_kick_1.move1()

    try:
        '''
        spin the node to execute the functions then shutdown the node when the functions are done or if there is an exception
        '''
        rclpy.spin(penalty_kick_1)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
        rclpy.shutdown()
    finally:
        print("Done")


if __name__ == '__main__':
    main()
