from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
duration = 0.15
th = 150

def front_obstacle(sensors):
    print(sensors[3])
    print(sensors[2])
    return sensors[3], sensors[1] > th

@event(robot.when_touched, [False, True])  # (..) button.
async def touched(robot):
    await robot.set_lights_rgb(0, 255, 0)
    await robot.play_note(Note.C5_SHARP, duration)

@event(robot.when_touched, [True, False])  # (.) button.
async def touched(robot):
    await robot.set_lights_rgb(0, 255, 0)
    await robot.play_note(Note.A5, duration)


@event(robot.when_play)
async def play(robot):
    print('Try moving your hand right in front of the front-central IR sensor')
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        await robot.play_note(sensors[3], Note.QUARTER)  # sensors[3] is the central front sensor.
        await robot.play_note(sensors[1], Note.QUARTER)

robot.play()
