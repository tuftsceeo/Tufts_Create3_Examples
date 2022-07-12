# Teleoperation & Teleprescence with iRobot™ Education's Create® 3

In this example, you will learn to control a Create®3 robot with the keyboard on your laptop. Being able to control the robot from a distance makes it teleoperational. To make it have telepresence, design a phone stand that can attach to the top of the Create®3 robot. Facetime or Zoom into your phone so you can see the environment where you are driving the robot. This allows you to feel present as your robot navigates around. 

## ROS2 Commands 

Great news, the package needed for teleoperation is already imbedded in the ROS 2 environment! We will just make slight changes to the parameters of the Create®3 robot and to the node arguments so that it all runs correctly. 

First, we need to disable the motion control safety features of the Create®3 robot. This will allow you to drive the robot in reverse. 

```
ros2 param set /[Namespace]/motion_control safety_override full
```

Then we can run the teleop_twist_keyboard package that is already in the ROS 2 environment. 

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --ros-args -r __ns:=/{Namespace}
```

## What You'll See

When you run this package & executable you will see the following information in your terminal. Now you can use the keys on your keyboard to remotely control your Create®3 robot. Happy Driving!

```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```
