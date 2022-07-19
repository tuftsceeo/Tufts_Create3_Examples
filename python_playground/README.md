# Python Playground Examples
This folder contains example python files. These files can be used in iRobot™ Education's Python Playground. The files include driving the robot using the bumpers, driving in a square, using it as an instrument, learning proportional integral control, and using IR sensors to avoid obstacles. They are meant to include many different components of the capabilities within python playground. Use these examples to write your own files to run on the Create®3 Educational Robot. To do so, open the example .py file in this GitHub and copy the code (top left icon that looks like two square). Paste the code in the python playground (use any prewritten example they give, delete the text and paste the new code). Since the python syntax may be a bit different than what you're used to, there are some notes on some common aspects of confusion below. 


## Events:
If a function is "decorated" by @event, then once the "event" is triggered, all of those functions will occur simultaneously. For example, if multiple functions are decorated as "@event(robot.when_play)," then whenever "robot.play()" is written in the script, it will trigger all functions with that event tag. In functions used with events, you must use "async." This is telling the script that we don't necessarily want to run that function right now. We want to call it when we want all the events to happen together, not in a partiuclar order. In other words, it is "asyncronous." Because of this, we don't use return values with event functions. Since all the event functions happen at the same time, we would never care to use the return vlaue of one of them.

Whenever you want a function to be decorated by @event, use async in front of def. Since we want to run these functions when the event tag is called, it is an asynchronous function, thus never running when we get to it in the script. For example:
```
@event(robot.when_play)
  async def move(robot):
    {code}

   
   
@event(robot.when_play)
  async def color(robot):
    {code}

    
robot.play
```

The robot.play line starts the robot's event system. It triggers all the functions decorated with @event(robot.when_play). Additionally, it will listen for other events like the ones associated with sensors. See max_obstacles.py, instrument.py, or bumper_control.py for full example. Each function that is triggered will run "in parallel," meaning they will all run at the same time. Normally, python scripts will run each function in the order that it reads. The events allow us to run functions at the same time or out of order, instead of one by one. 

Since the robot uses a few different sensors, now we can take advantage of all of them at once. We don't have to wait to run the bumpers if we are using the IR sensors. 

We can use the normal "def" format if we want to run that function to access its return value later. See max_obstacles.py as an example.

## Async:
You can use async with or without an event decorator. This is when you have a function that you don't want to run immediately in the script (such as the normal "def" function) or run when the event is called. Async is used with a function that is called later, usually in an event function. Async functions can have return values. If you want to run a function where it is in the script, use def. If you want it to run asynchronously and run when you call it, used async def. For example:
```
async def forward(robot):
    await robot.set_wheel_speeds(speed, speed)

async def sensors(robot):
    sensors = (await robot.get_ir_proximity()).sensors
    avg = sum(sensors)/len(sensors)
    return avg
    
@event(robot.when_play)
async def play(robot):
    await forward(robot)

robot.play()
```
(see max_obstacles.py for full example)

## Class:
A function in a class is called a method. Each method receives the 'self' parameter. In the example below, the parameter "name" is stored in the property "self.name". You can have multiple classes in a script. They can be used to organize the functions in your code. For example:
```
class my_robot:
    def __init__(self, name):
        self.name = name
    def message(self, color):
        print('I am ' + str(color))      
```

## Methods:
The robot's methods are all called with await. Methods are lines that tell the robot to do specific actions. It basically just means wait for this to occur until called. For example:
```
robot.set_wheel_speeds(speed, speed)
# or
robot.set_lights_rgb(255, 0, 0)
# or
robot.arc(Robot.DIR_LEFT, large_angle, arc_speed)
```
Methods should follow the await term such as:
```
await robot.set_wheel_speeds(speed, speed)
```

## Cheat Sheet:
• async doesn't have return values

• async used for events and used with await

• def and async def can have return values

• events happen simultaneously

• events don't use return values
