Will be more filled out, but adding notes for now:
- organized by instruction for each topic (event, async, etc) 


# Events:
If a function is "decorated" by @event, then once the "event" is triggered, all of those functions will occur simultaneously. For example, if multiple functions are decorated as "@event(robot.when_play)," then whenever "robot.play()" is written in the script, it will trigger all functions with that event tag. In functions used with events, you must use "async." This is telling the script that we don't necessarily want to run that function right now. We want to call it when we want all the events to happen together, not in a partiuclar order. In other words, it is "asyncronous." 

async:

class:
A function in a class is a method.
Each method receives the 'self' parameter.


It is important that an event function, is declated as async.

robot.play starts the robot's event system. It will do 2 things:
1. Trigger all the functions decorated with @event(robot.when_play).
2. Start listening for other events, such as the ones triggered by the robot's sensors.
3. trigger all the async functions decoreated with the @event(robot.when_play)

Any event (in this case we are using the robot.when_play event) can trigger multiple tasks that will run in parallel.
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
