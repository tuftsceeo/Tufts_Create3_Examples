Will be more filled out, but adding notes for now:
- organized by instruction for each topic (event, async, etc) 


events:


async:


A function in a class is a method.
Each method receives the 'self' parameter.

# Functions decorated with @event triggered by events.
A robot.when_play event is triggered when the robot.play() method is called.
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
