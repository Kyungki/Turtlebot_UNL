## Catkin Workspace
[Catkin](http://wiki.ros.org/catkin) is a CMAKE-based build system used to build ROS packages.

## Catkin Setup
We will be setting up a Catkin Workspace on **BOTH** the *Turtlebot* and *Master* computers using the following steps.

1. Open a new terminal
    1. Create a new directory in your home folder:
        * `mkdir -p ~/catkin_ws/src`
    2. CD to that directory:
        * `cd ~/catkin_ws/src`
    3. Initialize the workspace:
        * `catkin_init_workspace`
    4. Build the empty catkin workspace
        * `cd ~/catkin_ws`
        * `catkin_make`
    5. Source the setup.sh file to load the environment setup
        * `source ~/catkin_ws/devel/setup.sh`

