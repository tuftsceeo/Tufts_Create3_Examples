from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())

@event(robot.when_play)
async def play(robot):
    for i in range(4):
        await robot.move(10)
        await robot.turn_left(90)

                
async def square_2(robot):
    distance = 16
    await robot.navigate_to(0, distance)
    await robot.navigate_to(distance, distance)
    await robot.navigate_to(distance, 0)
    await robot.navigate_to(0, 0)
        
@event(robot.when_play)
async def play(robot):
    while True:
        await robot.set_lights_rgb(0, 255, 0)
        await robot.wait(0.3)
        await robot.set_lights_rgb(0, 0, 255)
        await robot.wait(0.3)
    await square_2(robot)

robot.play()

