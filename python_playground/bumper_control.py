'''
bumper_control.py
Tufts Create®3 Educational Robot Example
by Kate Wujciak
This file can be used in Python Playground. It allows you to tap the buttons and bumpers to control your robot's movements. 
'''
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note


'''
This initializes the robot. It allows us to call its methods (forward, turn, etc.) and define events with the @event decorator (robot.when_bumped, etc.).
'''
robot = Create3(Bluetooth())

'''
These are some variables I set to use later in the script. 
'''
speed = 10
arc_speed = 4
large_angle = 65
small_angle = 15

'''
This event function indicates that if the left bumper is hit the robot travels along an arc and changes its color to red. 
The "robot.when_bumped" indicates that we want to access the bumper sensors. As seen in the next function,
The bumper is defined by the [True, False] argument. If the left side reads True, it is the left bumper. If the right side reads True, it is the right
bumper. As a reminder (see README for moore instructions), every function with the event decorator will be called simultaneously when robot.play()
is run at the end of the script. To change the color, adjust the RGB values in the robot.set_lights_rgb(r,g,b) method. The robot.arc method has
three parameters: direction (Robot.DIR_LEFT), angle of arc (in degrees), and arc speed (in cm/s). 
'''
@event(robot.when_bumped, [True, False])
async def bumped(robot):
    await robot.set_lights_rgb(255, 0, 0)
    await robot.arc(Robot.DIR_LEFT, large_angle, arc_speed)
    await robot.wait(0.3)

'''
This function is essentially the same as the one above, except going right when the right bumper is hit and changing LED to green.
'''
@event(robot.when_bumped, [False, True])
async def bumped(robot):
    await robot.set_lights_rgb(0, 255, 0)
    await robot.arc(Robot.DIR_RIGHT, large_angle, arc_speed)
    await robot.wait(0.3)
    
@event(robot.when_touched, [True, False])  # (.) button
async def touched(robot):
    print('(.) button touched')
    await robot.set_lights(Robot.LIGHT_SPIN, Color(255, 100, 0))
    await robot.arc(Robot.DIR_LEFT, small_angle, arc_speed)


@event(robot.when_touched, [False, True])  # (..) button
async def touched(robot):
    print('(..) button touched')
    await robot.set_lights(Robot.LIGHT_SPIN, Color(100, 255, 0))
    await robot.arc(Robot.DIR_RIGHT, small_angle, arc_speed)

    
async def forward(robot):
    await robot.set_lights(Robot.LIGHT_SPIN, Color(255, 255, 0))
    await robot.set_wheel_speeds(speed, speed)


@event(robot.when_bumped, [])
async def move(robot):
    # This function will not be called again, since it never finishes.
    # Only task that are not currenctly running can be triggered.
    while True:
        await forward(robot)
        await robot.wait(0.3)



robot.play()
