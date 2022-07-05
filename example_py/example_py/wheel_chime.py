#Wheel Chime - Publisher, Subscriber, Action Example
#Maddie Pero 
#Project Create 

import sys
import rclpy
from rclpy.node import Node

#import messages & interfaces 

from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import WheelStatus 
from irobot_create_msgs.msg import AudioNote
from rclpy.action import ActionClient
from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.msg import AudioNoteVector 
from irobot_create_msgs.msg import AudioNote 
from builtin_interfaces.msg import Duration 


#define commonly used audio notes for easy access later on 

class AudioNotes():                       
    def __init__(self):
    	self.audionote1 = AudioNote(frequency=440, max_runtime= Duration(sec=1, nanosec=0))
    	self.audionote2 = AudioNote(frequency=880, max_runtime= Duration(sec=1, nanosec=0))


#define the node and its attributes
#this node will subscribe to the wheel status topic, publish to the audio note vector topic, and send an action to play an audio note sequence based off the wheel status
#must publish to the audio note vector topic because that is where the audio note sequence action gets its vector from

class Wheel_Subscriber(Node):
    
    #define the subscriber, publisher & action client 
              
    def __init__(self, namespace: str = '/JonSnow'):
    	super().__init__('Wheel_Sub')
    	self.subscription = self.create_subscription(WheelStatus, namespace + '/wheel_status',self.listener_callback, qos_profile_sensor_data)
    	self._action_client = ActionClient(self, AudioNoteSequence, namespace + '/audio_note_sequence')
    	self.publisher = self.create_publisher(AudioNoteVector, namespace + '/cmd_audio',10)
    	self.audio_note_vector = AudioNoteVector()
    	self.an = AudioNotes()
    	
    	# this function will get called whenever the computer recieves a message from the wheel status topic
    	
    def listener_callback(self, msg: WheelStatus):  
    	
    	#self.get_logger().info('I heard: "%s"' % msg) 
    	self.send_goal(msg, msg.wheels_enabled)
    	
    	#when the wheel status message is recieved the desired audio note will be sent to the Create 
	
    def send_goal(self, msg, wheels_enabled: bool):
    	
    	print("Wheels Are Enabled:", msg.wheels_enabled)
    	
    	# when the wheels are enabled play a lower frequency sound 
    	
    	if wheels_enabled == True : 
    		print('play low freq')
    		self.audio_note_vector.notes = [self.an.audionote1]
    		self.publisher.publish(self.audio_note_vector)
    		self._action_client.note_sequence = self.audio_note_vector
    	
    	# when the wheels become disabled play a higher frequency sound
    		
    	else :
    		print('play high freq')
    		self.audio_note_vector.notes = [self.an.audionote2]
    		self.publisher.publish(self.audio_note_vector)
    		self._action_client.note_sequence = self.audio_note_vector
    	goal_msg = AudioNoteSequence.Goal()
    	self._action_client.wait_for_server()
    		
    	return self._action_client.send_goal_async(goal_msg)

	#define what happens when the script is stopped 
	
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
