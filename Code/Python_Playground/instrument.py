'''
instrument.py
Tufts Create®3 Educational Robot Example
by Kate Wujciak
This file can be used in Python Playground. It allows you to tap the buttons and trigger different IR sensors (6 in total) to play different notes. 
'''
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

'''
This initializes the robot. It allows us to call its methods (play_note, set_lights_rgb, etc.) and define events with the @event decorator 
(robot.when_touched, etc.).
'''
robot = Create3(Bluetooth())
duration = 0.15

'''
This functions tells the robot to play a specific note when the right button is touched. It also changes the color. We use the eventt decorator 
because we always want the button to be "activated." The [False, True] parameter indicates that this function is for the right button.
'''
@event(robot.when_touched, [False, True])  # (..) button.
async def touched(robot):
    await robot.set_lights_rgb(0, 255, 0)
    await robot.play_note(Note.C5_SHARP, duration)

'''
This function is essentially the same as above, but it is for the left button (notice the [True, False] parameter).
'''
@event(robot.when_touched, [True, False])  # (.) button.
async def touched(robot):
    await robot.set_lights_rgb(0, 255, 0)
    await robot.play_note(Note.A5, duration)

'''
This function is for the IR sensors. It is not as complicated as it looks. In a while loop, we get the IR sensor data in an array with 6 items (uncomment
the print(sensors) to see what it looks like). Because it is likely that many sensors will be triggered but we can only play one note at a time, I want to
know which sensor is reading the highest value (object is closest to that sensor). I'm figuring out the highest value (max_sensor) then I'm figuring out
which sensor that value is associated with (max_ind). max_ind will tell me the index of the highest sensor. Since indices start at 0, I'm adding 1. Then,
depending on which sensor is the highest, I use a series of if statements to play a note depending on the sensor with the highest reading. 
'''
@event(robot.when_play)
async def play(robot):
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        max_sensor = max(sensors)
        max_ind = int(sensors.index(max_sensor)) +1 
        #print(sensors)
        #print('max val is ' + str(max_sensor))
        #print('index is ' + str(max_ind))
        if max_ind == 1:
            await robot.play_note(Note.A4, duration)
        if max_ind == 2:
            await robot.play_note(Note.B4, duration)
        if max_ind == 3:
            await robot.play_note(Note.C4, duration)
        if max_ind == 4:
            await robot.play_note(Note.D4, duration)
        if max_ind == 5:
            await robot.play_note(Note.E4, duration)
        if max_ind == 6:
            await robot.play_note(Note.F4, duration)

'''
This triggers all the functions with the event decorator. 
'''
robot.play()
