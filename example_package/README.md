# Example Package 

This folder contains an example package to run ROS 2 Python client library scripts. There are 5 scripts. An example of how to publish to a topic, an example of how to subscribe to a topic, two examples of how to send actions to the robot (two because both of us are stubborn and wanted to figure it out in different ways), and one eample of how to do all three at once. Subscribing, publishing and sending actions are the basic skills you will need to control the iRobot™ Education's Create®3 Educational Robot.

### To use the scripts in this package (or add your own scripts) follow the below instructions. 

1. From your home directory navigate to the workspace containing the package 
```
cd ./Tufts_Create3_Examples/example_package
```
2. Change the Namepsace in each file to match the Namespace of your Create®3 robot.
3. Build the package & wait for a few seconds until it says "1 package finished"
```
colcon build --packages-select example_package
```
4. Source the package
```
source install/setup.bash
```
5. Run the scripts. Keep an eye on both your robot & the terminal to figure out what the scripts do. 
```
ros2 run example_package [executable name]
```
The executable names are : **subscribe **, **publish** , **action** , **action2** , and **combined**. 
We recomend that you run them in that order. Each executable name will run the corresponding python script. For descriptions of what each script does, please check out the comments in the code. 

For more information about how to build and run packages in ROS 2 go [here](https://katewujciak.wixsite.com/projectcreate/running-py-files-with-ros2).
