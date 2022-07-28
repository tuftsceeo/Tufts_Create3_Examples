## Using Airtable to control iRobot™ Education's Create®3 Robot 

A more advanced way to do telerobotics is to use Airtable to remotely control your Create®3 robot. Airtable is a cloud collaboration service that lets you store
information in a visual environment. The Airtable API allows you to get information from the table and put it into your python code with a few simple
commands. In this example, I created an Airtable with 6 different cells. The cells correspond to x,y and z angular and linear velocities. By putting numbers into these
cells we can then change the linear and angular velocities of the wheels of the robot. In other words, drive it around. 

<img width="605" alt="image" src="https://user-images.githubusercontent.com/60265399/180259959-05a121c5-35c8-4c38-95cf-0afb44b71a00.png">

This is an image of what my Airtable looked like. By putting 3 in for all angular velocity cells I was able to make the robot spin in a circle. 

If you share your Airtable with your friends, they could drive your Create®3 robot from the other side of the world. 

### Use this code

1. Add this python file to the example package folder on your virtual machine. 
2. Add a new executable name in the entry points section of the setup.py file. 
3. Build, install and then run the package as explained in the example package [README](https://github.com/brianabouchard/Tufts_Create3_Examples/blob/main/example_package/README.md). 
