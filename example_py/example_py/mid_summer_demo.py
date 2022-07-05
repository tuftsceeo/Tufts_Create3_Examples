#Mid Summer Demo - Bump_Sound
#Maddie Pero 
#Project Create 

import sys
import rclpy
from rclpy.node import Node

#import messages & interfaces 

from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector


from irobot_create_msgs.msg import AudioNote
from rclpy.action import ActionClient
from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.msg import AudioNoteVector 
from irobot_create_msgs.msg import AudioNote 
from builtin_interfaces.msg import Duration 


#define commonly used audio notes for easy access later on 

class AudioNotes():                       
    def __init__(self):
    	self.audionote1 = AudioNote(frequency=440, max_runtime= Duration(sec=0, nanosec=500000000))
    	self.audionote2 = AudioNote(frequency=660, max_runtime= Duration(sec=0, nanosec=500000000))
    	self.audionote3 = AudioNote(frequency=880, max_runtime= Duration(sec=0, nanosec=500000000))
    	self.audionote4 = AudioNote(frequency=1000, max_runtime= Duration(sec=0, nanosec=500000000))
    	self.audionote5 = AudioNote(frequency=1100, max_runtime= Duration(sec=0, nanosec=500000000))
#define the node and its attributes
#this node will subscribe to the hazard detection topic, create a publisher to the audio note sequence topic, and send an action to play an audio note
#must publish to the audio note vector topic because that is where the audio note sequence action gets its vector from

class Bump_Sound(Node):
    
    #define the subscriber, publisher & action client 
              
    def __init__(self, namespace: str = '/JonSnow'):
    	super().__init__('bump_sound')
    	
    	self.subscription = self.create_subscription(HazardDetectionVector, namespace + '/hazard_detection',self.listener_callback, qos_profile_sensor_data)
    	
    	self._action_client = ActionClient(self, AudioNoteSequence, namespace + '/audio_note_sequence')
    	
    	self.publisher = self.create_publisher(AudioNoteVector, namespace + '/cmd_audio',10)
    	self.audio_note_vector = AudioNoteVector()
    	self.an = AudioNotes()
    	
    	# this function will get called whenever the computer recieves a message from the hazard detection sensor
    def listener_callback(self, msg):  
    	for detection in msg.detections:
    		det = detection.header.frame_id
    		
    		if det!= "base_link":
    			print(det)
    			if det =="bump_right":
    				self.audio_note_vector.notes = [self.an.audionote1]
    				self.publisher.publish(self.audio_note_vector)
    				self._action_client.note_sequence = self.audio_note_vector
    				self.send_goal()
    			elif det == "bump_left":
    				self.audio_note_vector.notes = [self.an.audionote2]
    				self.publisher.publish(self.audio_note_vector)
    				self._action_client.note_sequence = self.audio_note_vector
    				self.send_goal()
    			elif det == "bump_front_left":
    				self.audio_note_vector.notes = [self.an.audionote3]
    				self.publisher.publish(self.audio_note_vector)
    				self._action_client.note_sequence = self.audio_note_vector
    				self.send_goal()				
    			elif det == "bump_front_right":
    				self.audio_note_vector.notes = [self.an.audionote4]
    				self.publisher.publish(self.audio_note_vector)
    				self._action_client.note_sequence = self.audio_note_vector
    				self.send_goal()
    			elif det == "bump_front_center":
    				self.audio_note_vector.notes = [self.an.audionote5]
    				self.publisher.publish(self.audio_note_vector)
    				self._action_client.note_sequence = self.audio_note_vector
    				self.send_goal()
	
	#will send the action to the robot 
    def send_goal(self):
    	
    	goal_msg = AudioNoteSequence.Goal()
    	self._action_client.wait_for_server()
    		
    	return self._action_client.send_goal_async(goal_msg)

	#define what happens when the script is stopped 
	
def main(args=None):
    rclpy.init(args=args)	
    bump_sound = Bump_Sound()
    
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
