'''
unfinished
'''
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
duration = 0.15
th = 150


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
        max_sensor = max(sensors)
        max_ind = int(sensors.index(max_sensor)) +1 
        print(sensors)
        print('max val is ' + str(max_sensor))
        print('index is ' + str(max_ind))
        if max_ind == 1:
            await robot.play_note(Note.A4, .1)
        if max_ind == 2:
            await robot.play_note(Note.B4, .1)
        if max_ind == 3:
            await robot.play_note(Note.C4, .1)
        if max_ind == 4:
            await robot.play_note(Note.D4, .1)
        if max_ind == 5:
            await robot.play_note(Note.E4, .1)
        if max_ind == 6:
            await robot.play_note(Note.F4, .1)

robot.play()
