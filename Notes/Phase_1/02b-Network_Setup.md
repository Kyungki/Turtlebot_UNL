## Network Setup
Connect to a Wireless, Ethernet, or Cellular network using the Network Manager (upper right corner of desktop).
![](Resources/01/wificonf.png)

1. Find the current IP for the Turtlebot
```bash
hostname -I
```

2. Add the network parameters to your BashRC
```bash
echo export ROS_MASTER_URI=http://$(hostname -I):11311 >> ~/.bashrc
echo export ROS_IP=$(hostname -I) >> ~/.bashrc
echo export ROS_HOSTNAME=$(hostname -I) >> ~/.bashrc
echo export ROS_HOME=~/.ros >> ~/.bashrc
```
