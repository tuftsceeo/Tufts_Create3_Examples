## Jupyter Notebook Examples

The beauty of a Juypter Notebook is that it simplifies the coding process by bypassing the compiling, building, package hastle. Don't worry you will get to that later.

There are 5 scripts in this jupyter notebook folder. An example of how to publish to a topic, an example of how to subscribe to a topic, two examples of how to send actions to the robot (two because both of us are stubborn and wanted to figure it out in different ways), and one eample of how to do all three at once. Subscribing, publishing and sending actions are the basic skills you will need to control the iRobot™ Education's Create®3 Educational Robot.

## Get Started
0. If you have not downloaded the zip file of the repo, go back to the [home page](https://github.com/brianabouchard/Tufts_Create3_Examples) and follow the directions on that README.md to download all the necessary files. Once that is done, continue to Step 1.  

1. Install pip on your machine:
```
sudo apt-get install python-pip
```
2. Install Jupyter Notebook in Terminal:
```
python3 -m pip install jupyter
```
2. In terminal, navigate from your home directory to this directory. 
```
cd ./Tufts_Create3_Examples/jupyter_notebook_examples
```
4. Open the files in a browser with the following command. If the web browser does not automatically populate click the provided link. 
```
jupyter notebook
``` 
5. In the browser page, click on a script you wish to run. 
6. Change the [Namepsace] in the file to match the Namespace of your Create®3 robot.
7. Click on the cell to so that is highlighted in blue & press the run button to run the script. 
8. To close the notebook go back to terminal & use ctrl c or command c. 

For descriptions of what each script will do, check out the comments in the code. 

Write your own code by adding a cell at the bottom of any file, typing out your code, selecting the cell so that it turns blue, and pressing the run button.
