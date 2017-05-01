/*
 * rosserial Publisher Example
 * Publishes the Arduino millis to the topic /arduino/millis
 */

/* Include the ROS library */
#include <ros.h>

/* Include the ROS message for an unsigned 32-bit integer
   Arduino's millis() command returns a `long`,
   which is an unsigned 32-bit integer */
#include <std_msgs/UInt32.h>

ros::NodeHandle  nh;

std_msgs::UInt32 millis_msg;
ros::Publisher pub_millis("/arduino/millis", &millis_msg);


void setup()
{
  // Initialize ROS Node
  nh.initNode();
  // Advertise the publisher topic
  nh.advertise(pub_millis);
}

void loop()
{
  // Set the millis_msg data to the millis() output
  millis_msg.data = millis();
  // Tell ROS to publish the data
  pub_millis.publish( &millis_msg );
  // Send the ROS message to the ROS Master
  nh.spinOnce();
  
  delay(5);
}
