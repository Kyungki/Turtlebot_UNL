## Network Setup
Connect to a Wireless, Ethernet, or Cellular network using the Network Manager (upper right corner of desktop).
![](Resources/01-wificonf.png)

1. Find the current IP for the Turtlebot by opening a terminal on the turtlebot:
```bash
hostname -I
```

2. Find the current IP for the Master by opening a terminal on the master computer:
```bash
 hostname -I
```

3. Add the network parameters to your BashRC for the turtlebot computer
```bash
echo export ROS_MASTER_URI=http://$(hostname -I):11311 >> ~/.bashrc
echo export ROS_IP=$(hostname -I) >> ~/.bashrc
echo export ROS_HOSTNAME=$(hostname) >> ~/.bashrc
echo export ROS_HOME=~/.ros >> ~/.bashrc
```

4. Add the network parameters to your BashRC for the master computer:  
Replace `IP_OF_TURTLEBOT` with the ip found in step `1`
```bash
echo export ROS_MASTER_URI=http://IP_OF_TURTLEBOT:11311 >> ~/.bashrc
echo export ROS_IP=$(hostname -I) >> ~/.bashrc
echo export ROS_HOSTNAME=$(hostname) >> ~/.bashrc
echo export ROS_HOME=~/.ros >> ~/.bashrc
```

5. Load the new environment variables above into your active terminal window:
```bash
source ~/.bashrc
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

# Troubleshooting
If your laptop cannot connect to the network, ensure that you installed [the proprietary drivers](http://askubuntu.com/questions/22118/can-i-install-extra-drivers-via-the-command-prompt).

If you continue to have issues, check out the [ROS Network Setup Page](http://wiki.ros.org/ROS/NetworkSetup)
