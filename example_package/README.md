# Example Package 

This folder contains an example package to run ROS2 Python client library scripts. There are 5 scripts. An example of how to publish to a topic, an example of how to subscribe to a topic, two examples of how to send actions to the robot (two because both of us are stubborn and wanted to figure it out in different ways), and one eample of how to do all three at once. Subscribing, publishing and sending actions are the basic skills you will need to control the  iRobot® Create® 3 Educational Robot. Master them and you can master the Create® 3. 

To use the scripts in this package (or add your own scripts) follow the below instructions. 

1. Download the zip file of this repo & unzip it. 
2. Remove the example_package folder and place it in your home directory. (It doesn't matter what you do with the other files. Delete them for all I care.)
3. 
```
colcon build --packages-select example_package
```

4. 
```
source install/setup.bash
```
5. Change the namepsace in each file to match your own. 

6. 
```
ros2 run example_package [executable name]
```
The executable names are : subscribe , publishe , action , action2 , and combined. They can be found in the setup.py file. 

For more information about how to build and run packages in ROS2 go [here](https://katewujciak.wixsite.com/projectcreate/running-py-files-with-ros2).
