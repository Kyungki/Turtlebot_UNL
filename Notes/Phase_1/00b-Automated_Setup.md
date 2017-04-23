## Automated Setup
This tool setups the environment in which the tools run. It assumes that the [laptop setup](/Notes/Phase_1/02-Master_Setup.md) has been completed.  

## Installation
Simply use [git](http://rogerdudler.github.io/git-guide/
) to copy this entire repository onto your *Turtlebot* and *Master* computer and follow the other commands:
```bash
git clone --recursive https://github.com/dabit-industries/turtlebot-houston ~/turtlebot-houston
```

If you'd like the most recent version of the code on this site, you can update it with following commands:
```
cd ~/turtlebot-houston
git pull
```

## Setup
These commands are to be run on both the **Master** and **Turtlebot** computer
1. On the command line, navigate to the repository folder:
    1. `cd ~/turtlebot-houston`
2. Run the environment setup program:
    1. `source ~/turtlebot-houston/Setup/automated/automate.sh`
        * You should see: `Environment setup successful`
3. Run our utility to check, copy, and compile all the code:
    1. `dabit-setup-utility install`
        * This code requires some interaction, please follow along in the terminal
        * This may take a while as it is getting all the dependencies and building all the code
        * This code moves the existing `~/catkin_ws` and `~/workspace` to `~/backup/old_workspace_DD-MM-YY_hh-mm-ss`
    2. On the **Master** Computer, set your *MASTER_IP*:
        * `change_master IP_OF_TURTLEBOT`
4. Test a code example:
    1. In a new terminal:
        * type: `roslaunch turtlebot_bringup minimal.launch`
    2. In a new terminal:
        * type: `rosrun turtlebot_houston roscpp_hello_world`
        * You should see the following in your terminal:
        
            ```
            [ INFO] [1492930878.699859775]: Hello from ROS node /roscpp_example
            ```

## Running ROSPY Sample Code:
* `rosrun turtlebot_houston rospy_hello_world.py`
* `rosrun turtlebot_houston rospy_opencv.py`
    * You need to run `roslaunch turtlebot_bringup 3dsensor.launch` before running this code
* `rosrun turtlebot_houston rospy_example_class`

## Running ROSCPP Sample Code:
* `rosrun turtlebot_houston roscpp_hello_world`
* `rosrun turtlebot_houston roscpp_opencv`
    * You need to run `roslaunch turtlebot_bringup 3dsensor.launch` before running this code
* `rosrun turtlebot_houston roscpp_pcl_example`
* `rosrun turtlebot_houston roscpp_publisher`
* `rosrun turtlebot_houston roscpp_subscriber`

## Running GUI Code:
1. `rosrun rqt_turtlebot_slam rqt_turtlebot_slam`

## Running ROS Arduino:
1. `roslaunch turtlebot_houston arduino.launch`

## Running Wanderer Code:
**WORK-IN-PROGRESS**

## TROUBLESHOOTING:
1. *rqt* applications don't run
    * I.E. `rqt_image_view` doesn't run
    * `sudo apt remove python-qt4`
