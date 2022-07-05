# Example Package 

This folder contains an example package to run ROS2 Python client library scripts. 

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
5. 
```
ros2 run example_package example_executable
```

For more information about how to build and run packages in ROS2 go [here](https://katewujciak.wixsite.com/projectcreate/running-py-files-with-ros2).
