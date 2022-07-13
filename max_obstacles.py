from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 10
th = 150


async def forward(robot):
    await robot.set_lights(Robot.LIGHT_SPIN, Color(255, 255, 0))
    await robot.set_wheel_speeds(speed, speed)


async def backoff(robot):
    await robot.set_lights(Robot.LIGHT_BLINK, Color(255, 50, 60))
    await robot.move(-20)
    await robot.turn_left(45)


def front_obstacle(sensors):
    print(sensors[3])
    return sensors[3] > th

@event(robot.when_bumped, [True, False])
async def bumped(robot):
    print('Bumped in the LEFT')


@event(robot.when_bumped, [False, True])
async def bumped(robot):
    print('Bumped in the RIGHT')


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
                print('Create has run into ' + str(count) + ' obstacle(s).')
                print('Create has been bumped ' +str(l_bump)+ ' times on the left and ' + str(r_bump)+' times on the right')
    print('Too many obstacles --> Mission Terminated')
    await robot.move(0)

robot.play()
