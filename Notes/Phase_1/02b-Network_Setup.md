## Network Setup
Connect to a Wireless, Ethernet, or Cellular network using the Network Manager (upper right corner of desktop).
![](Resources/01/wificonf.png)

1. Find the current IP for the Turtlebot by opening a terminal on the turtlebot:
```bash
hostname -I
```

2. 
Find the current IP for the Master
```bash
 hostname -I
```

2. Add the network parameters to your BashRC
```bash
echo export ROS_MASTER_URI=http://$(hostname -I):11311 >> ~/.bashrc
echo export ROS_IP=$(hostname -I) >> ~/.bashrc
echo export ROS_HOSTNAME=$(hostname) >> ~/.bashrc
echo export ROS_HOME=~/.ros >> ~/.bashrc
```

## Network Testing
ROS requires completely free network connectivity between the Turtlebot and the Master computer.

The first test is the basic Ping between the Master and the Turtlebot:
- `ping IP_OF_TURTLEBOT`

You should see the following returned if the Turtlebot is on the network:
```
PING IP_OF_TURTLEBOT (IP_OF_TURTLEBOT) 56(84) bytes of data.
64 bytes from IP_OF_TURTLEBOT: icmp_seq=1 ttl=64 time=66.8 ms
```

# Troubleshoot
If your laptop cannot connect to the network, ensure that you installed [the proprietary drivers](http://askubuntu.com/questions/22118/can-i-install-extra-drivers-via-the-command-prompt).

If you continue to have issues, check out the [ROS Network Setup Page](http://wiki.ros.org/ROS/NetworkSetup)
