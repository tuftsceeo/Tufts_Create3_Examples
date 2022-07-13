from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())

async def square_1(robot):
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
    await square_1(robot)
    print('New square!')
    await square_2(robot)

robot.play()

