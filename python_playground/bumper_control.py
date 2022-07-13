'''
header
'''
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())

speed = 4
large_angle = 65
small_angle = 15

#@event(robot.when_touched, [True, True])
#async def go(robot):
#    await forward(robot)

@event(robot.when_bumped, [True, False])
async def bumped(robot):
    await robot.set_lights_rgb(255, 0, 0)
    await robot.arc(Robot.DIR_LEFT, large_angle, speed)
    await robot.wait(0.3)


@event(robot.when_bumped, [False, True])
async def bumped(robot):
    await robot.set_lights_rgb(0, 255, 0)
    await robot.arc(Robot.DIR_RIGHT, large_angle, speed)
    await robot.wait(0.3)
    
@event(robot.when_touched, [True, False])  # (.) button
async def touched(robot):
    print('(.) button touched')
    await robot.set_lights(Robot.LIGHT_SPIN, Color(255, 100, 0))
    await robot.arc(Robot.DIR_LEFT, small_angle, speed)


@event(robot.when_touched, [False, True])  # (..) button
async def touched(robot):
    print('(..) button touched')
    await robot.set_lights(Robot.LIGHT_SPIN, Color(100, 255, 0))
    await robot.arc(Robot.DIR_RIGHT, small_angle, speed)

    
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
