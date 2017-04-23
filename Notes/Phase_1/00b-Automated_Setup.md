## Automated Setup
This tool is meant to automate all the tutorials.  
This is only for use if you want to jump right into using the code without following any tutorials and learning any background. 

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
    1. `bash turtlebot-houston/Setup/automated/automate.sh`
        * You should see: `Environment setup successful`
3. Run our utility to check, copy, and compile all the code:
    1. `dabit-setup-utility install`
        * This code requires some interaction, please follow along in the terminal
        * This may take a while as it is getting all the dependencies and building all the code
        * This code moves the existing `~/catkin_ws` and `~/workspace` to `~/old_workspace`
        
4. Test a code example:
    1. In a new terminal:
        * type: `roslaunch turtlebot_bringup minimal.launch`
    2. In a new terminal:
        * type: `roslaunch turtlebot_houston test.launch`
        * You should see the following in your terminal:
        
            ```
            // Everything checks out
            ```
    3. In the same terminal, 
