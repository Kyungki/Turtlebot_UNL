## RQT_GUI

1. Create Catkin (Build) Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
2. Clone tutorial repo into workspace
```bash
https://github.com/dabit-industries/turtlebot-houston
```
3. Go to directory containing GUI code:
```bash
cd ~/catkin_ws/src/turtlebot-houston/catkin_ws/src/turtlebot-rqt-slam
```

4. Install dependencies
```bash
sudo apt install python-qt-binding python3-pyqt5 -y
```
5. Build the Workspace
```bash
cd ~/catkin_ws/src/turtlebot-houston/catkin_ws
catkin_mke
Clean rqt cache
rm ~/.config/ros.org/rqt_gui.ini
```
6. Source the bash files
```bash
 source ~/.bashrc
 source ~/catkin_ws/src/turtlebot-houston/catkin_ws/devel/setup.bash
 ```
7. Run the interface
```bash
 rosrun rqt_turtlebot_slam rqt_turtlebot_slam
```
