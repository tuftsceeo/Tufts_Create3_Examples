## Tufts_Create3_Examples

Welcome! In this repo we (some undergrad mechanical engineering students at Tufts University) aim to help you master iRobot™ Education's Create®3 Robot through code examples and design challenges. To control the Create®3 robot we will use ROS 2 Galactic and the Python Client Library. Please follow the directions written [here](https://iroboteducation.github.io/create3_docs/setup/ubuntu2004/) to set up ROS 2 if you have not already.  

If you run into errors search [here](https://katewujciak.wixsite.com/projectcreate/parallels) (step 12) for answers.

## Get Started
1. Download this repository on your virtual machine. 
    - Click the green "Code" button then "Download ZIP"
2. Unzip the repository & move it to the **Home** directory of your virtual machine. Double check it is still named "Tufts_Create3_Examples"
3. Follow the READMEs in the other folders to test out the code 

## Generally Important Information 

In any file, if a piece of code is written inside {} brackets it means you should fill it out with your unique namespace/package name/workspace etc. Do not include the {} when you run the code.

For example, if the namespace of my Create® 3 robot is JonSnow. Then I would want to modify the code accordingly. For example:
```
ros2 topic echo /{Namespace}/battery_state
```
becomes
```
ros2 topic echo /JonSnow/battery_state
```
when you run it in your terminal.
