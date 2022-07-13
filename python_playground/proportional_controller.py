#need to comment
#proportional controller - slows down as it approaches the wall
# should be P = P0 + error*Kp

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth()) 
max_speed = 40 #edit this value so that it stops as close to the wall as possible
async def forward(robot,speed):
    await robot.set_wheel_speeds(speed,speed)

async def sensors(robot):
    sensors = (await robot.get_ir_proximity()).sensors
    avg = sum(sensors)/len(sensors)
    return avg

@event(robot.when_play)
async def when_play(robot):
    while True:
        error = await sensors(robot)
        print(sensor)
        if error != 0: #adding an if/else statement so that we don't divide by 0
            speed = max_speed/error #error is tricky w/ IR sensors bc the vale grows as it gets closer to wall 
        #because of this the easiest way to reduce speed as we approach the wall is to divide
        #desired speed by the sensor value
        else:
            speed = max_speed
        await forward(robot,speed)

robot.play()

