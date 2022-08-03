## Invisble Springs 

This project is an example of creating a proportional controller for the iRobot™ Education's Create®3 Robot.

The goal of the script is to have one robot maintain a set distance from the robot driving in front of it using the IR sensors. Typically a proportional controller
follows the format of P = P0 + error*Kp.

![Invisible Springs GIF](https://user-images.githubusercontent.com/60265399/182504673-8662b67f-873a-4e95-8c84-8faffcda2840.gif)


## Get Started
0. Download and unzip the repository as previously [directed](https://github.com/brianabouchard/Tufts_Create3_Examples/blob/main/README.md).

1. Navigate from your home directory to this directory 
```
cd Tufts_Create3_Examples-main/Projects/Invisible_Springs
```
2. Edit the namespace in the files to match your robot's namespace
3. Run the scripts with 
```
python3 invisible_springs.py
```
