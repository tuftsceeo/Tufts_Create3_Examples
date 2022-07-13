
# Events:
If a function is "decorated" by @event, then once the "event" is triggered, all of those functions will occur simultaneously. For example, if multiple functions are decorated as "@event(robot.when_play)," then whenever "robot.play()" is written in the script, it will trigger all functions with that event tag. In functions used with events, you must use "async." This is telling the script that we don't necessarily want to run that function right now. We want to call it when we want all the events to happen together, not in a partiuclar order. In other words, it is "asyncronous." 

Whenever you want a function to be decorated by @event, use async in front of def. For example:
```
@event(robot.when_play)
  async def move(robot):
    {code}

   
   
@event(robot.when_play)
  async def color(robot):
    {code}

    
robot.play
```

The robot.play line starts the robot's event system. It triggers all the functions decorated with @event(robot.when_play). Additionally, it will listen for other events like the ones associated with sensors. See max_obstacles.py, instrument.py, or bumper_control.py for full example. Each function that is triggered will run "in parallel," meaning they will all run at the same time. Normally, python scripts will run each function in the order that it reads. The events all us to run functions at the same time or out of order, instead of one by one. 

Since the robot uses a few different sensors, now we can take advantage of all of them at once. We don't have to wait to run the bumpers if we are using the IR sensors. 

We can use the normal "def" format if we want to run that function to access its return value later. See max_obstacles.py as an example.

# Async:
You can use async without an event decorator. This is when you have a function that you don't want to run immediately in the script (normal "def" function) or run when the event is called. Async is used with a function that is called later, usually in an event function. Async functions do not have return values. Use the normal "def" format if you want a return value from a function. For example:
```
async def forward(robot):
    await robot.set_wheel_speeds(speed, speed)
    
@event(robot.when_play)
async def play(robot):
    await forward(robot)

robot.play()
```
(see max_obstacles.py for full example)

# Class:
A function in a class is called a method. Each method receives the 'self' parameter. In the example below, the parameter "name" is stored in the property "self.name". For example:
```
class my_robot:
    def __init__(self, name):
        self.name = name
    def message(self, color):
        print('I am ' + str(color))      
```

# Methods:
The robot's methods are all called with await. Methods are lines that tell the robot to do specific actions. It basically just means wait for this to occur until called.


# Cheat Sheet:
• async doesn't have return values

• def has return values

• async used for events and used with await

• events happen simultaneously
