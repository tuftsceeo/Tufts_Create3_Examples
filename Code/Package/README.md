# Example Package 

This folder contains an example package to run ROS 2 Python client library scripts. These scripts are meant to teach the basics of subscribing, publishing and sending actions which are all the necessary commands to control the iRobot™ Education's Create®3 Educational Robot.

## Get Started
0. Download and unzip the repository as previously [directed](https://github.com/brianabouchard/Tufts_Create3_Examples/blob/main/README.md).

1. From your home directory navigate to the workspace containing the package 
```
cd Tufts_Create3_Examples-main/Package
```
2. Change the namepsace in each file to match the namespace of your Create®3 robot.
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
The executable names are : **subscribe**, **publish** , **action** , **action2** , and **combined**. 
We recomend that you run them in that order. Each executable name will run the corresponding python script (which can be found in the Package sub folder). For descriptions of what each script does, please check out the comments in the code. 

For more information about how to build and run packages in ROS 2 go [here](https://katewujciak.wixsite.com/projectcreate/running-py-files-with-ros2).
