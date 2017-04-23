## Automated Setup
This tool is meant to automate all the tutorials.  
This is only for use if you want to jump right into using the code without following any tutorials and learning any background. 

## Installation
Simply clone this entire repository onto your *Turtlebot* and *Master* computer and follow the other commands:
1. Make a new directory in your home titled `Dabit`:
    1. `mkdir ~/Dabit`
2. Clone this entire repository:
    1. `git clone --recursive https://github.com/dabit-industries/turtlebot-houston ~/Dabit/turtlebot-houston`

## Setup
These commands are to be run on both the **Master** and **Turtlebot** computer
1. CD Into the repository:
    1. `cd ~/Dabit/turtlebot-houston`
2. Run the environment setup program:
    1. `bash ~/Dabit/turtlebot-houston/Setup/automated/automate.sh`
        * You should see: `Environment setup successful`
3. Run our utility to check, copy, and compile all the code:
    1. `dabit-setup-utility install`
        * This code requires some interaction, please follow along in the terminal
        * This may take a while as it's getting all the dependencies and building all the code
        * This code moves `~/catkin_ws` and `~/workspace` to `~/old_workspace`
4. Test a code example:
    1. In a new terminal, launch the `turtlebot_bringup minimal.launch`
        * `roslaunch turtlebot_bringup minimal.launch`
    2. In a new terminal, launch the `turtlebot_houston test.launch`
        * `roslaunch turtlebot_houston test.launch`
        * You should see the following in your terminal:
        
            ```
            // Everything checks out
            ```
    3. In the same terminal, 
