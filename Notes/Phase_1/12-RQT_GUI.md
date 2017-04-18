## RQT_GUI

Create Catkin (Build) Workspace
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/src
 catkin_init_workspace
Clone Dabit code into workspace
 git clone https://github.com/dabit-industries/turtlebot-rqt-slam
Install dependencies
 sudo apt install python-qt-binding python3-pyqt5
Build Workspace
cd .. catkin_ws
 catkin_make
Clean rqt cache
 rm ~/.config/ros.org/rqt_gui.ini
Source the bash files
 source ~/.bashrc
 source ~/catkin_ws/devel/setup.bash
Run the interface
 rosrun rqt_turtlebot_slam rqt_turtlebot_slam
