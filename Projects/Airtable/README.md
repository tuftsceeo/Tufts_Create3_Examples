## Using Airtable to control iRobot™ Education's Create®3 Robot 

A more advanced way to do telerobotics is to use Airtable to remotely control your Create®3 robot. Airtable is a cloud collaboration service that lets you store
information in a visual environment. The Airtable API allows you to get information from the table and put it into your python code with a few simple
commands. In this example, I created an Airtable with 6 different cells. The cells correspond to x,y and z angular and linear velocities. By putting numbers into these
cells we can then change the linear and angular velocities of the wheels of the robot. In other words, drive it around. 


This is an image of what my Airtable looked like. By putting 3 in for all angular velocity cells I was able to make the robot spin in a circle. 
<img width="605" alt="image" src="https://user-images.githubusercontent.com/60265399/180259959-05a121c5-35c8-4c38-95cf-0afb44b71a00.png">

![Airtable GIF](https://user-images.githubusercontent.com/60265399/182436305-3d006fec-a3c0-4f68-9093-a693245db41f.gif)


If you share your Airtable with your friends, they could drive your Create®3 robot from the other side of the world. Similar to the basic telerobot project, build a phone stand that can attach to the top of the Create®3 robot. Facetime or Zoom into your phone so you can see the environment where you are driving the robot and make it have telepresence.

## Get Started
0. Download and unzip the repository as previously [directed](https://github.com/brianabouchard/Tufts_Create3_Examples/blob/main/README.md).

1. Navigate from your home directory to this directory 
```
cd Tufts_Create3_Examples-main/Projects/Airtable
```
2. Edit the namespace, Airtable base ID and API Key to match your's.
3. Run the script with 
```
python3 airtable_telerobot.py
```
