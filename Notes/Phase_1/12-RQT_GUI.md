## RQT_GUI

1. Create Catkin (Build) Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
2. Clone tutorial repo into workspace
```bash
git clone https://github.com/dabit-industries/turtlebot-rqt-slam
```
3. Install dependencies
```bash
sudo apt install python-qt-binding python3-pyqt5
```
4. Build the Workspace
```bash
cd .. catkin_ws
catkin_make
Clean rqt cache
rm ~/.config/ros.org/rqt_gui.ini
```
5. Source the bash files
```bash
 source ~/.bashrc
 source ~/catkin_ws/devel/setup.bash
 ```
6. Run the interface
```bash
 rosrun rqt_turtlebot_slam rqt_turtlebot_slam
```
