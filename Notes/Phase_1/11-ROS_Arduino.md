## ROS Arduino
This tutorial provides the necessary setup for using an Arduino with ROS.
In this Tutorial, you will learn:
    * Rosserial
    * communication between the Arduino and rosserial
    * configuring udev rules to automatically find the arduino
    * creating a launch file to launch rosserial connected to your arduino
    * receiving data from the arduino
    * sending data to the arduino


## Prequisites
This section **requires** the *catkin_ws* to be initialized.
[Please click here to learn how to initialize the catkin workspace](08-Catkin_Workspace.md)

## Installing ROSSERIAL
These are the basic steps for installing ROSSERIAL and the arduino libraries.
For detailed installation steps, [click here](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

1. ROSSERIAL is a package used to bridge ROS with devices using a serial connection  
    * `sudo apt install ros-kinetic-rosserial* -y` 
2. [Download the latest Arduino IDE](https://www.arduino.cc/en/Main/Software) if you don't have it installed
    * Start Arduino at least once before continuing
3. CD to your libraries folder in your Arduino Sketchbook (typically ~/Arduino)
    * `cd ~/Arduino/libraries`
4. Run rosserial to generate ros_lib:
    * `rosrun rosserial_arduino make_libraries.py .`

## Connecting to the Arduino

1. Finding the Arduino's mount point:
    * `sudo dmesg -C`
    * unplug the Arduino, replug the Arduino
    * `sudo dmesg`
    * Find the Device Connected info and look for:  
        `[ 2857.315493] usb 1-6: FTDI USB Serial Device converter now attached to ttyUSB0`
        
2. Create a new package in your *catkin_ws* for housing our launch files
    * `cd ~/catkin_ws/src`
    * `catkin_create_pkg turtlebot_houston`
    * `cd turtlebot_houston`
    * `mkdir launch`

3. Make an `arduino.launch` file in the ~/catkin_ws/src/turtlebot_houston/launch folder:
    * `gedit arduino.launch`
```xml
<launch>
  <node pkg="rosserial_server" type="serial_node" name="rosserial_timing">
    <param name="port" value="/dev/ttyUSB0" />
  </node>
</launch>
```

4. Build your package using `catkin`
    * `cd ~/catkin_ws`
    * `catkin_make`

## Verifying connection to the Arduino
1. Launch arduino.launch
    * `source ~/catkin_ws/devel/setup.sh`
    * `roslaunch turtlebot_houston arduino.launch`

## Sending data to the Arduino
Follow our [A simple Arduino subscriber](11b-Arduino_Subscriber.md) tutorial

## Additional Tutorials
ROSSERIAL has recently improved their tutorials, and have a very wide selection of examples.
[Check out the ROSSerial_Arduino Tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials)

## Troubleshooting
When using USB 3, the port may be `ttyACM0`. If so, change the port in the `arduino.launch` and change the permissions to the port`sudo chmod 777 /dev/ttyACM0`
