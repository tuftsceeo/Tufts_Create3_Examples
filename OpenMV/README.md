## OpenMV Cam and iRobot™ Education's Create®3 Robot 

In these two examples we use an OpenMV Cam programed to communicate r,g,b values over serial and the USB-C port on the Create®3 Robot to direct the motion of the robot. 

The stop_signs script uses the camera to recognize red, yellow and green road signs. At each color sign the robot is directed to turn in a different direction. Below
is a sped up video demo. 

https://user-images.githubusercontent.com/60265399/181576743-21592599-682c-4d3e-ba1e-7aade335f23a.mp4

The linefollow script uses the camera to follow a black line on a white table top. At blue intersections along the black line the robot will stop and wait 3 seconds before proceeding.
Below is a sped up video demo.

https://user-images.githubusercontent.com/60265399/181576776-47f304c1-086f-4cab-9cd2-cae05ab89835.mp4

## Get Started
0. Download and unzip the repository as previously directed.
1. Navigate from your home directory to this directory.
```
cd Tufts_Create3_Examples/OpenMV
```
2. Change the namespace & IP address of the robot in the scripts 
3. Run either script with the command 
```
python3 [script_name].py
```

The script names are "stop_signs" and "line_follow" . 
