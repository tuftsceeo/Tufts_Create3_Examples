'''
drive_square.py
Tufts Create®3 Educational Robot Example
by Kate Wujciak
This file can be used in Python Playground. It allows you to drive in a square. It has two different square written.  
'''
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

'''
This initializes the robot. It allows us to call its methods (move, navigate, etc.) and define events with the @event decorator (robot.when_play, etc.).
'''
robot = Create3(Bluetooth())

'''
This function does not have an event decorator because we don't want to run it yet. It is a simple function that allows the robot to move in a square.
In a for loop, it has the robot move forward 10 cm and turn 90º left four times. 
'''
async def square_1(robot):
    for i in range(4):
        await robot.move(10)
        await robot.turn_left(90)
'''
This function also drive the robot in a square but using a different strategy. It has a set distance and will navigate to different "coordinate points"
which results in a square. This function also doesn't have an event decorator. 
'''                
async def square_2(robot):
    distance = 16
    await robot.navigate_to(0, distance)
    await robot.navigate_to(distance, distance)
    await robot.navigate_to(distance, 0)
    await robot.navigate_to(0, 0)
  
'''
This function runs both square functions in order. This allows us to run both separately so the robot doesn't get confused. When it's called by robot.play()
it will run square_1, print 'New square!,' then run square_2. 
'''
@event(robot.when_play)
async def play(robot):
    await square_1(robot)
    print('New square!')
    await square_2(robot)

'''
This calls all the functions with the event decorator, which in this case, in just one function that calls back to two other functions. 
'''
robot.play()

