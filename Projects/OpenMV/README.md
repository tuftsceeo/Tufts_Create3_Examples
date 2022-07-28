## OpenMV Cam and iRobot™ Education's Create®3 Robot Crossover Episode

In these two examples we use an OpenMV Cam programed to communicate r,g,b values over serial and the USB-C port on the Create®3 Robot to direct the motion of the robot. 

The road_signs script uses the camera to recognize red, yellow and green road signs. At each color sign the robot is directed to turn in a different direction. Below
is a sped up video demo. 

https://user-images.githubusercontent.com/60265399/181616368-7205a090-473a-49e8-a43f-761d8551019d.mp4


The linefollow script uses the camera to follow a black line on a white table top. At blue intersections along the black line the robot will stop and wait 3 seconds before proceeding.
Below is a sped up video demo.

https://user-images.githubusercontent.com/60265399/181576776-47f304c1-086f-4cab-9cd2-cae05ab89835.mp4

## Get Started

0. Download and unzip the repository as previously [directed](https://github.com/brianabouchard/Tufts_Create3_Examples/blob/main/README.md).

1. Navigate from your home directory to this directory 
```
cd Tufts_Create3_Examples-main/Projects/OpenMV
```
2. Edit the namespace in the files to match your robot's namespace
3. Run the scripts with 
```
python3 [script_name].py
```

The script names are "road_signs" and "line_follow" . 
