'''
combined_audio_bump.py
Tufts Create®3 example
by Maddie Pero 

This script is an example of how to combine a subscriber, publisher and an action client under one class. 
'''

import sys
import rclpy
from rclpy.node import Node

'''
import messages/actions/interfaces/variable types
'''
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import AudioNote
from rclpy.action import ActionClient
from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.msg import AudioNoteVector 
from irobot_create_msgs.msg import AudioNote 
from builtin_interfaces.msg import Duration  

class AudioNotes():
    '''
    Define a class of commonly used audio notes that we can reference later. Each audio note
    requires a frequency and max run time.
    '''
    def __init__(self):
    	self.audionote1 = AudioNote(frequency=440, max_runtime= Duration(sec=0, nanosec=500000000))
    	self.audionote2 = AudioNote(frequency=660, max_runtime= Duration(sec=0, nanosec=500000000))
    	self.audionote3 = AudioNote(frequency=880, max_runtime= Duration(sec=0, nanosec=500000000))
    	self.audionote4 = AudioNote(frequency=1000, max_runtime= Duration(sec=0, nanosec=500000000))
    	self.audionote5 = AudioNote(frequency=1100, max_runtime= Duration(sec=0, nanosec=500000000))

class Bump_Sound(Node):
    '''
    This class contains a publisher, a subscriber and an action client. It will take in data from
    a the hazard detection topic, publish to the audio note sequence topic, and send a goal to play
    an audio note. We are defining a class "Bump_Sound" which is a subclass of Node. 
    '''
              
    def __init__(self, namespace: str = '/[Namespace]'):
    	'''
    	We initialize the class by calling the Node constructor then naming our node 'bump_sound'
    	'''
    	super().__init__('bump_sound')
    	
    	'''
    	Then we initialize the subscriber, publisher and action client
    	'''
    	self.subscription = self.create_subscription(HazardDetectionVector, namespace + '/hazard_detection',self.listener_callback, qos_profile_sensor_data)
    	self.publisher = self.create_publisher(AudioNoteVector, namespace + '/cmd_audio',10)
    	self._action_client = ActionClient(self, AudioNoteSequence, namespace + '/audio_note_sequence')
    	
    	'''
    	rename Create3 messages so that they can be later manipulated with methods
    	'''
    	self.audio_note_vector = AudioNoteVector()
    	self.an = AudioNotes()
    	
    def listener_callback(self, msg):  
        '''
        Whenever the computer recieves a message from the hazard detection sensors this function will be called
        and any actions in this function will be executed
        '''
        '''
        first we parse the message to determine which bumper was hit. 
        Then depending on what the message change we change the goal accordingly.
        '''
        for detection in msg.detections:
        	det = detection.header.frame_id
        	
        	if det!= "base_link":
    		    print(det)
    		    if det =="bump_right":
    		        '''
    		        in order to play an audio note we need to publish an audio note to the audio note vector 
    		        topic. Then we can send a goal to play the audio note vector that is currently in the topic
    		        '''
    		        self.audio_note_vector.notes = [self.an.audionote1]
    		        self.publisher.publish(self.audio_note_vector)
    		        self._action_client.note_sequence = self.audio_note_vector
    		        self.send_goal()
    		    elif det == "bump_left":
    		        '''
    		        repeat that process but with different notes for each bumper that is hit
    		        '''
    		        self.audio_note_vector.notes = [self.an.audionote2]
    		        self.publisher.publish(self.audio_note_vector)
    		        self._action_client.note_sequence = self.audio_note_vector
    		        self.send_goal()
    		    elif det == "bump_front_left":
    		        '''
    		        repeat that process but with different notes for each bumper that is hit
    		        '''
    		        self.audio_note_vector.notes = [self.an.audionote3]
    		        self.publisher.publish(self.audio_note_vector)
    		        self._action_client.note_sequence = self.audio_note_vector
    		        self.send_goal()				
    		    elif det == "bump_front_right":
    		        '''
    		        repeat that process but with different notes for each bumper that is hit
    		        '''
    		        self.audio_note_vector.notes = [self.an.audionote4]
    		        self.publisher.publish(self.audio_note_vector)
    		        self._action_client.note_sequence = self.audio_note_vector
    		        self.send_goal()
    		    elif det == "bump_front_center":
    		        '''
    		        repeat that process but with different notes for each bumper that is hit
    		        '''
    		        self.audio_note_vector.notes = [self.an.audionote5]
    		        self.publisher.publish(self.audio_note_vector)
    		        self._action_client.note_sequence = self.audio_note_vector
    		        self.send_goal()
				
    def send_goal(self):
    	'''
	this function defines the goal, waits for the action server to be available
	and then sends the goal to the action server. It gets called in the timer_callback()
	function whenever we need to send a goal to the robot
	'''
    	goal_msg = AudioNoteSequence.Goal()
    	self._action_client.wait_for_server()
    		
    	return self._action_client.send_goal_async(goal_msg) 
	
def main(args=None):
    '''
    Initializes ROS2 and creates an instance of 
    'Bump_Sound'
    '''
    rclpy.init(args=args)	
    bump_sound = Bump_Sound()
    
    '''
    run code in the class until therer is a keyboard interrupt. then shutdown the node
    '''
    try:
        rclpy.spin(bump_sound)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        bump_sound.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
