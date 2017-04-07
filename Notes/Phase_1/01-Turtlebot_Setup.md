# Turtlebot Setup

## Overview
TurtleBot 2 is an open robotics platform designed for education and research on state of art robotics. It is also a powerful  tool to teach and learn ROS (Robot Operating System) and make the most of this cutting edge techonology. Equipped with a 3D sensor, it can map and navigate indoor enviroments. Due to the Turtlebot's modularity, you can attach your own sensors, electronics, and mechanics easily.

## Physical Assembly:

![](Resources/01/explode_view_01.jpg)

## Testing Kobuki
Launch the GUI to check out the Kobuki status:
 ```bash
 roslaunch turtlebot_dashboard turtlebot_dashboard.launch
``` 
If everything is OK, it should look like this:
![](Resources/01/turtlebot_dashboard.png)

You can test drive the Kobuki base by using the Keyboard teleop launch file located in the turtlebot_teleop package
```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```
![](Resources/01/turtlebot_keyboard_teleop.png)
