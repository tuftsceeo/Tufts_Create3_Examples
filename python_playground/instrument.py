from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
duration = 0.15
th = 150

def front_obstacle(sensors):
    if sensors[1] != 0:
        await robot.play_note(Note.C5_SHARP, duration)
    if sensors[3] != 0:
        await robot.play_note(Note.A5_SHARP, duration)
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
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        print(sensors)
        await robot.play_note(sensors[1], Note.QUARTER)
        await robot.play_note(sensors[2], Note.QUARTER)
        await robot.play_note(sensors[3], Note.QUARTER)
        await robot.play_note(sensors[4], Note.QUARTER)
        await robot.play_note(sensors[5], Note.QUARTER)
        await robot.play_note(sensors[6], Note.QUARTER)

robot.play()
