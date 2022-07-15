'''
invisible_springs.py
Tufts Create®3 Educational Robot Example
by Maddie Pero
This file can be used in Python Playground. It is proportional controller. The robot's speed will change based on how close it is to the wall. 
P = P0 + error*Kp
'''
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

'''
This initializes the robot. It allows us to call its methods (set_wheel_speeds, get_ir_proximity, etc.) and define events with the @event decorator 
(robot.when_play, etc.).
'''
robot = Create3(Bluetooth()) 
max_speed = 40 #edit this value so that it stops as close to the wall as possible

'''
This function does NOT have an event decorator because we don't want it to be called with the other functions. Since we want the IR sensor to be 
"activated" at all times, we use an event decorator. But, we only want it to go forward when we tell it to. It uses a method where you can set
specific wheel speed for each wheel. The async in front implies that we do not want it to run that function when it gets to it in the script. 
We want to call it later. 
'''
async def forward(robot,speed):
    await robot.set_wheel_speeds(speed,speed)

'''
This async function determines and returns the average of all 6 sensors. 
'''
async def sensors(robot):
    sensors = (await robot.get_ir_proximity()).sensors
    avg = sum(sensors)/len(sensors)
    return avg

'''
This event function is the controller. Proportional control is a type of feedback system during which an adjustment is applied to the controlled 
variable which is proportional to the difference between the desired value (set point which is max_speed) and the current measured value (process variable
which is speed) ->(this difference is the error). 

In this case, our error is calculcated in the sensors function above, which returns the average of the sensors. Since we can't divide by zero,
an if/else statement is added. In theory, the goal of a P controller is to achieve an error of 0. Thus, when error is zero (else statement), the current 
speed (process variable) is set to our desired value (set point which is max_speed). Error is tricky w/ IR sensors bc the value grows as it gets closer to 
wall. Because of this the easiest way to reduce speed as we approach the wall is to divide desired speed by the sensor value.
'''
@event(robot.when_play)
async def when_play(robot):
    while True:
        error = await sensors(robot)
        print(sensor)
        if error != 0: 
            speed = max_speed/error
        else:
            speed = max_speed
        await forward(robot,speed)

'''
This command tells all the events to start, which triggers the rest of the script.
'''
robot.play()
