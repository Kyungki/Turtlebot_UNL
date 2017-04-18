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
    * `sudo apt install ros-kinetic-rosserial*`
2. [Download the latest Arduino IDE](https://www.arduino.cc/en/Main/Software) if you don't have it installed
    * Start Arduino at least once before continuing
3. CD to your libraries folder in your Arduino Sketchbook (typically ~/Arduino)
    * `cd ~/Arduino/libraries`
4. Run rosserial to generate ros_lib:
    * `rosrun rosserial_arduino make_libraries.py .`


## Sending data to the Arduino
As an example to send data to the Arduino, we will use the blink example.o
More informatino [can be pulled up, here](http://wiki.ros.org/rosserial_arduino/Tutorials/)

1. Upload the following code to your Arduino:

```c
/* 
 * rosserial Subscriber Example
 * Turns 
 */

#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

void messageCb( const std_msgs::Bool& msg){
  digitalWrite(13, msg.data);   // set the pin state to the message data
}

ros::Subscriber<std_msgs::Bool> sub("/arduino/led", &messageCb );

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
```

## Connecting to the Arduino

1. Finding the Arduino's mount point:
    * `sudo dmesg -C`
    * unplug the Arduino, replug the Arduino
    * `sudo dmesg`
    * Find the Device Coonected info and look for:  
        `[ 2857.315493] usb 1-6: FTDI USB Serial Device converter now attached to ttyUSB0`

2. Launch file
```xml
<launch>
  <node pkg="rosserial_server" type="serial_node" name="rosserial_timing">
    <param name="port" value="/dev/ttyUSB0" />
  </node>
</launch>
```

## Communicating with the Arduino
1. Type
    * `rostopic list`
2. With the above example,and ROSSERIAL running, we can see in rostopic our Arduino topic:
    * __/arduino/led__
3. You can send data to this topic by using rostopic pub:
    * `rostopic pub /arduino/led std_msgs/Bool True`
    * `rostopic pub /arduino/led std_msgs/Bool False`

## Additional Tutorials
ROSSERIAL has recently improved their tutorials, and have a very wide selection of examples.
[Check out the ROSSerial_Arduino Tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials)

## Troubleshooting
