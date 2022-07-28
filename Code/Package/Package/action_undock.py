'''
This file was heavily influenced by Sawyer Paccione's action undock file:
https://github.com/paccionesawyer/Create3_ROS2_Intro/blob/main/individual_examples/action_undock.py
action_undock.py
Tufts Create® 3 Educational Robot Example
This file is a simple action client file that will undock the robot if it is docked. 
'''

'''
These statements allow the Node class to be used and actions to be performed. 
'''
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

'''
This statement imports the undock action.
'''
from irobot_create_msgs.action import Undock

'''
Input your namespace here as a global variable. 
'''
namespace = '[Namespace]'

class UndockingActionClient(Node):
    '''
    This is an action client. Action clients send goal requests to action servers.
    We are defining a class "UndockingActionClient" which is a subclass of Node. 
    '''

    def __init__(self):
        '''
        We initialize the class by calling the Node constructor then
        naming our node 'dockservo_action_client'
        '''
        super().__init__('undocking_action_client')
        
        '''
        Here we initiate a new action server. We include where to add the action client
        (self), the type of action (DockServo), and the action name ('dock').
        '''  
        print('Initiating a new action server...')
        self._action_client = ActionClient(self, Undock, namespace + '/undock')

    def send_goal(self):
        '''
        This is the goal message.
        '''
        goal_msg = Undock.Goal()
        print('Goal Message: ' + str(goal_msg))
        
        '''
        This method waits for the action server to be available.
        '''

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    '''
    Initializes ROS2 and creates an instance of 
    'UndockingActionClient'
    '''
    rclpy.init(args=args)
    undock_client = UndockingActionClient()
    
    '''
    Sends a goal to the server.
    '''
    print('Action server available. Sending undock goal to server.')
    future = undock_client.send_goal()
    
    '''
    When an action server accepts or rejects the goal, future is completed.
    '''
    rclpy.spin_until_future_complete(undock_client, future)
    print('Robot is undocking. Shutting down action undock client node.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
