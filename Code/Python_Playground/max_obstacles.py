'''
max_obstacles.py
Tufts Create®3 Educational Robot Example
by Kate Wujciak
This file can be used in Python Playground. Once it senses an obstacle, it will stop, go backwards, turn, then go forward again. This process will repeat
until one of two things happen: either a bumper is hit, or it runs into a maximum number of obstacles. 
'''
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

'''
This initializes the robot. It allows us to call its methods (forward, turn, etc.) and define events with the @event decorator (robot.when_bumped, etc.).
Global variables are also initialized here including speed and threshold. 
'''
robot = Create3(Bluetooth())
speed = 10
th = 150
stop = False

'''
This function tells the robot, if any bumper is hit (notice how (robot.when_bumped, []) has no False parameters, indicating [True, True]). The global stop
line means that if this function is triggered, it will stop the script completely. The LED will also turn red. As a reminder (see README for more 
instructions), every function with the event decorator will be called simultaneously when robot.play()
'''
@event(robot.when_bumped, [])
async def bumped(robot):
    global stop
    await robot.set_lights_rgb(255, 0, 0)
    stop = True 
    await robot.stop()

'''
This function (which does not have an event decorator) will not occur until forward(robot) is called. It sets the wheel speed (going forward) and changes
the color. 
'''
async def forward(robot):
    await robot.set_lights(Robot.LIGHT_SPIN, Color(255, 255, 0))
    await robot.set_wheel_speeds(speed, speed)

'''
This function is similar to above, but goes backwards then turns instead of going forwards. It also blinks a specific color.
'''
async def backoff(robot):
    await robot.set_lights(Robot.LIGHT_BLINK, Color(255, 50, 60))
    await robot.move(-20)
    await robot.turn_left(45)

'''
This function determines if the front center sensor (sensor[3]) is above the threshold of 150. If it is, it will return that value. 
'''
def front_obstacle(sensors):
    print(sensors[3])
    return sensors[3] > th

'''
This event function will tell you if the left bumper was hit. The "robot.when_bumped" indicates that we want to access the bumper sensors. 
As seen in the next function, the bumper is defined by the [True, False] argument. If the left side reads True, it is the left bumper. 
If the right side reads True, it is the right bumper. 
'''
@event(robot.when_bumped, [True, False])
async def bumped(robot):
    print('Bumped in the LEFT')

'''
This function is the same as above but for the right bumper.
'''
@event(robot.when_bumped, [False, True])
async def bumped(robot):
    print('Bumped in the RIGHT')

'''
This event function will tell the robot when it senses an obstacle and what to do. First, a counter is initialized. This will allow us to count how 
many obstacles the robot senses. Then, we enter a while loop while the number of obstacles remains under 2. Thus, once two obstacles are sensed, 
the robot will not enter the while and will not continue its actions. When it first enters the while loop, the forward function is called, telling
the robot to go forward. Then, if the front sensor function returns a value above the threshold, it will carry out a series of functions. These
functions include the backoff function (backing up and turning), playing a note, going forward, then playing a different note. A new if statement is
included which indicates if the front sensor sensed an obstacle, the counter will increase by 1. Then, when it starts the while loops again, the counter
will be at 1 and not 0, continuing until it senses two obstacles. After that, the robot will stop moving. 
'''
@event(robot.when_play)
async def play(robot):
    count = 0
    while count < 2:
        await forward(robot)
        sensors = (await robot.get_ir_proximity()).sensors
        if front_obstacle(sensors):
            await backoff(robot)
            await robot.play_note(Note.C5_SHARP, 1)
            await forward(robot)
            await robot.play_note(Note.A4, 1)
            if sensors[3] != 0:
                count += 1
                print('Create has sensed ' + str(count) + ' obstacle(s)')
        else:
            print('No obstacles currently detected.')
    print('Too many obstacles --> Mission Terminated')
    await robot.move(0)

'''
This command tells all the events to start, which triggers the rest of the script.
'''
robot.play()
