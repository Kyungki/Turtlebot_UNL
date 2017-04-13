
## Turtlebot Teleop
Now that you have configured the network connections and are able to ping between machines, you can control the turtlebot from the master using the Keyboard Teleop launch file.

- Open a new terminal on the master laptop
  - `roslaunch turtlebot_teleop keyboard_teleop.launch`

You should see the IP_OF_TURTLEBOT near the top of the terminal window.
![](Resources/04-turtlebot_keyboard_teleop_master.png)

You can test drive the Kobuki base by using the Keyboard teleop launch file located in the turtlebot_teleop package
```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```
![](Resources/04-turtlebot_keyboard_teleop.png)


## Turtlebot Follower
The Turtlebot has an additional program that can follow an entity in front of it.  
The follower program calculates the centroid of an observed object in front of it at a certain distance. If the centroid of the object is too far away it will drive forward, too close backward, and if offset to the side it will turn toward the centroid. 

1. Open a new terminal on the turtlebot laptop:
    * `roslaunch turtlebot_follower follower.launch`


Using the configuration GUI for ROS you can modify the parameters of the follower.
1. In a new terminal:
  - `rosrun rqt_reconfigure rqt_reconfigure`

2. Select camera/follower on the reconfigure gui.

3. Play with the sliders and see how the parameters effect the Turtlebot's behavior.

![](Resources/04-turtlebot_follow_reconfigure.png)
