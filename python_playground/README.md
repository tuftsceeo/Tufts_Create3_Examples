Will be more filled out, but adding notes for now:
- organized by instruction for each topic (event, async, etc) 


# Events:
If a function is "decorated" by @event, then once the "event" is triggered, all of those functions will occur simultaneously. For example, if multiple functions are decorated as "@event(robot.when_play)," then whenever "robot.play()" is written in the script, it will trigger all functions with that event tag. In functions used with events, you must use "async." This is telling the script that we don't necessarily want to run that function right now. We want to call it when we want all the events to happen together, not in a partiuclar order. In other words, it is "asyncronous." 

Whenever you want a function to be decorated by @event, use async in front of def. For example:
@event(robot.when_play)
  async def move(robot):
    ~code~
    ...
    ...
    ...
   
   
@event(robot.when_play)
  async def color(robot):
    ~code~
    ...
    ...
    ...
    
robot.play

The robot.play line starts the robot's event system. It triggers all the functions decorated with @event(robot.when_play). Additionally, it will listen for other events like the ones associated with sensors. See max_obstacles.py, instrument.py, or bumper_control.py for full example. Each function that is triggered will run "in parallel," meaning they will all run at the same time. Normally, python scripts will run each function in the order that it reads. The events all us to run functions at the same time or out of order, instead of one by one. 

# Async:
You can use async without an event decorator. This is when you have a function that you don't want to run immediately in the script (normal "def" function) or run when the event is called. Async is used with a function that is called later, usually in an event function. For example:
...
async def forward(robot):
    await robot.set_wheel_speeds(speed, speed)
    
@event(robot.when_play)
async def play(robot):
    await forward(robot)

robot.play()

(see max_obstacles.py for full example)

# Class:
A function in a class is called a method. Each method receives the 'self' parameter. In the example below, the parameter "name" is stored in the property "self.name". For example:
class my_robot:
    def __init__(self, name):
        self.name = name
    def message(self, color):
        print('I am ' + str(color))




An event triggered by the robot's sensors, such as its bumpers, can also run multiple tasks.
robot methods are all called with await.

new event (not robot.when_play) for each sensor
event for sensor, normal fnc for non-sensor related actions.

# notes:
when event is triggered, it will run those "events."
async def means "don't run this function yet"
we want only def when we want the return value later in the script.
on the other hand, async doesn't tend to have return values so they can just be
called whenever for that function to happen.
await means wait for this to occur until called.
async used for events
def used with return values
async used with await
events happen simultaneously
