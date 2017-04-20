/*
 * OpenCV Example using ROS and CPP
 */

// Include the ROS library
#include <ros/ros.h>

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV Window Name
static const std::string OPENCV_WINDOW = "Image window";

// Topics
static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
static const std::string PUBLISH_TOPIC = "/image_converter/output_video";

// Image Converter class
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Create a new ROS Subscriber
    image_sub_ = it_.subscribe(IMAGE_TOPIC, 1,
      &ImageConverter::imageCb, this);
    // Create a new ROS Publisher
    image_pub_ = it_.advertise(PUBLISH_TOPIC, 1);

    // Initialize an OpenCV Window
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    // Destroy OpenCV Window upon program exit
    cv::destroyWindow(OPENCV_WINDOW);
  }

  // Callback function when data is received from the subcriber
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  // Initialize the ROS Node "opencv_example"
  ros::init(argc, argv, "opencv_example");
  
  // Create a new string, append "Hello ROS!" to it, and print it to the terminal and ROS log file
  std::stringstream ss;
  ss << "Hello ROS!";
  ROS_INFO("%s", ss.str().c_str());

  // Initialize the ImageConverter class
  ImageConverter ic;

  // loop to communicate with ROS
  ros::spin();

  // Program successful
  return 0;
}
