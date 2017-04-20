## OpenCV with ROS using C++
Using OpenCV with ROS is possible using the CvBridge library.
This tutorial will show you how to get a message from an Image topic in ROS, convert it to an OpenCV Image, and manipulate the image.

## Prequisites
This example requires an image stream on the `/camera/rgb/image_raw` topic.
1. On the _turtlebot_, run 3dsensor.launch:
    1. `roslaunch turtlebot_bringup 3dsensor.launch`

This section **requires** the *catkin_ws* to be initialized and the *turtlebot_houston* package created.  
[Please click here to learn how to initialize the catkin workspace](08-Catkin_Workspace.md)


## Setting up the catkin package to build
For this tutorial, we will be using the *turtlebot_houston* catkin package we created.  
In order to enable building C++ files in our package, we need to edit the *CMakeLists.txt* and *package.xml* to build with our dependencies.  
1. Source your catkin workspace and cd to the *turtlebot_houston* package:
    1. `cd ~/catkin_ws/src/turtlebot_houston`
2. Edit your *package.xml* and add build and run dependencies:
    1. `gedit package.xml`
        * Near the bottom of the file, find the `<buildtool_depend>catkin</buildtool_depend>` line:

            ```xml
            ...
            <buildtool_depend>catkin</buildtool_depend>
            ...
            ```

        * Add the build_depend and run_depend tags for roscpp **underneath** the *buildtool_depend* line:

            ```xml
            <build_depend>roscpp</build_depend>
            <run_depend>roscpp</run_depend>
            ```

        * [The resulting package.xml](/Setup/catkin_ws/src/turtlebot_houston/package.xml)

			```xml
            <?xml version="1.0"?>
            <package>
              <name>turtlebot_houston</name>
              <version>0.0.0</version>
              <description>The turtlebot_houston package</description>

              <!-- One maintainer tag required, multiple allowed, one person per tag --> 
              <!-- Example:  -->
              <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
              <maintainer email="user@todo.todo">user</maintainer>


              <!-- One license tag required, multiple allowed, one license per tag -->
              <!-- Commonly used license strings: -->
              <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
              <license>TODO</license>

              <!-- ... -->

              <buildtool_depend>catkin</buildtool_depend>
              <build_depend>roscpp</build_depend>
              <run_depend>roscpp</run_depend>

              <!-- The export tag contains other, unspecified, tags -->
              <export>
                <!-- Other tools can request additional information be placed here -->

              </export>
            </package>
            ```

3. Edit your *CMakeLists.txt* and add your dependencies to it
    1. `gedit CMakeLists.txt`
        * Near the top of the file, find the `find_package(catkin REQUIRED)` line:

            ```
            ...
            find_package(catkin REQUIRED)
            ...
            ```

        * Replace that line with:

            ```
			find_package(catkin REQUIRED COMPONENTS
				roscpp
			)
            ```

        * Near the middle of the file, find the `catkin_package(` line:

            ```
            ...
            catkin_package(
            #  INCLUDE_DIRS include
            #  LIBRARIES turtlebot_houston
            #  CATKIN_DEPENDS other_catkin_pkg
            #  DEPENDS system_lib
            )
            ...
            ```

        * Replace that line with:

            ```
            catkin_package(
              INCLUDE_DIRS
              CATKIN_DEPENDS roscpp
            )
            ```

        * Near the middle of the file, find the `# include_directories(include)` line:

            ```
            # include_directories(include)
            ```

        * Replace that line with:

            ```
            include_directories(
              ${catkin_INCLUDE_DIRS}
            )
            ```

        * [The resulting CMakeLists.txt](/Setup/catkin_ws/src/turtlebot_houston/CMakeLists.txt)

            ```
			cmake_minimum_required(VERSION 2.8.3)
			project(turtlebot_houston)

			## Add support for C++11, supported in ROS Kinetic and newer
			# add_definitions(-std=c++11)

			## Find catkin macros and libraries
			## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
			## is used, also find other catkin packages
			find_package(catkin REQUIRED COMPONENTS
			  roscpp
			)

			#  ...

			###################################
			## catkin specific configuration ##
			###################################
			## The catkin_package macro generates cmake config files for your package
			## Declare things to be passed to dependent projects
			## INCLUDE_DIRS: uncomment this if you package contains header files
			## LIBRARIES: libraries you create in this project that dependent projects also need
			## CATKIN_DEPENDS: catkin_packages dependent projects also need
			## DEPENDS: system dependencies of this project that dependent projects also need
			catkin_package(
			  INCLUDE_DIRS
			  CATKIN_DEPENDS roscpp
			)

			###########
			## Build ##
			###########

			## Specify additional locations of header files
			## Your package locations should be listed before other locations
			include_directories(
			  ${catkin_INCLUDE_DIRS}
			)

			# ...

			## Declare a C++ executable
			## With catkin_make all packages are built within a single CMake context
			## The recommended prefix ensures that target names across packages don't collide
			# add_executable(${PROJECT_NAME}_node src/turtlebot_houston_node.cpp)

			add_executable(opencv_example src/opencv_example.cpp)
			target_link_libraries(opencv_example ${catkin_LIBRARIES})

			## ...
			```

4. Test your new package configuration is correct by running catkin_make:
    * `catkin_make --directory ~/catkin_ws --pkg turtlebot_houston`
    * Upon success, you should see:

        ```
        -- Configuring done
        -- Generating done
        -- Build files have been written to: /home/user/catkin_ws/build
        ####
        #### Running command: "make -j4 -l4" in "/home/user/catkin_ws/build/turtlebot_houston"
        ####
        ```

## Writing the first ROSCPP C++ code
Now we get to write our first c++ code to communicate with ROS.  
1. Create a new file titled *opencv_example.cpp* in your *src* folder
    1. `gedit ~/catkin_ws/src/turtlebot_houston/src/opencv_example.cpp`
    2. Start with the following *Hello World* code in gedit:

        ```c
        /*
         * OpenCV Example using ROS and CPP
         */

        // Include the ROS library
        #include <ros/ros.h>

        // Main function
        int main(int argc, char** argv)
        { 
          // Initialize the ROS Node "opencv_example"
          ros::init(argc, argv, "opencv_example");

          // Instantiate the ROS Node Handler as nh
          ros::NodeHandle nh;
          
          // Create a new string, append "Hello ROS!" to it, and print it to the terminal and ROS log file
          std::stringstream ss;
          ss << "Hello ROS!";
          ROS_INFO("%s", ss.str().c_str());
          
          // Program succesful
          return 0;
        } 
        ```

    3. Save and exit *opencv_example.cpp*
2. Edit your *CMakeLists.txt* and specify to build *opencv_example.cpp*:
    1. `gedit ~/catkin_ws/src/turtlebot_houston/CMakeLists.txt`
        * Near the middle of the file, find the `## Declare a C++ executable`:

            ```
            ...
            ## Declare a C++ executable
            ## With catkin_make all packages are built within a single CMake context
            ## The recommended prefix ensures that target names across packages don't collide
            # add_executable(${PROJECT_NAME}_node src/turtlebot_houston_node.cpp)
            ...
            ```

        * Below that line, add the following:

            ```
            add_executable(opencv_example src/opencv_example.cpp)
            target_link_libraries(opencv_example ${catkin_LIBRARIES})
            ```

    2. Save *CMakeLists.txt* and exit gedit
3. Compile your code using catkin_make:
    1. `catkin_make --directory ~/catkin_ws --pkg turtlebot_houston`
        * Upon success, you should see:

            ```
            ####
            #### Running command: "make -j4 -l4" in "/home/user/catkin_ws/build/turtlebot_houston"
            ####
            Scanning dependencies of target opencv_example
            [ 50%] Building CXX object turtlebot_houston/CMakeFiles/opencv_example.dir/src/opencv_example.cpp.o
            [100%] Linking CXX executable /home/user/catkin_ws/devel/lib/turtlebot_houston/opencv_example
            [100%] Built target opencv_example
            ```

4. Run your code using rosrun:
    1. `source ~/catkin_ws/devel/setup.sh`
    2. `rosrun turtlebot_houston opencv_example`
        * Upon success, you should see: `[ INFO] [1492726164.127098818]: Hello ROS!`

## Getting an Image from a ROS topic and using OpenCV
1. Set up your package dependencies:
    1. `gedit ~/CMakeLists.txt`
        * Replace `find_package(catkin REQUIRED COMPONENTS)` with:

            ```
            find_package(catkin REQUIRED COMPONENTS
              roscpp
              sensor_msgs
              std_msgs
              cv_bridge
              image_transport
            )

            find_package(OpenCV 2 REQUIRED)
            ```

        * Replace `catkin_package(` with:

            ```
            catkin_package(
              INCLUDE_DIRS
              CATKIN_DEPENDS roscpp
            )
            ```
        
        * Replace `include_directories(` with:
            
            ```
			include_directories(
			  ${catkin_INCLUDE_DIRS}
			  ${OpenCV_INCLUDE_DIRS}
			)
			```

		* Replace `target_link_libraries(opencv_example` with:

			```
            target_link_libraries(opencv_example ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
            ```

    2. `gedit package.xml`
        * Add the build_depend and run_depends under roscpp:

            ```
            <build_depend>roscpp</build_depend>
            <run_depend>roscpp</run_depend>
            <build_depend>cv_bridge</build_depend>
            <build_depend>image_transport</build_depend>
            <build_depend>libopencv-dev</build_depend>
            <build_depend>sensor_msgs</build_depend>
            <build_depend>std_msgs</build_depend>
            <run_depend>cv_bridge</run_depend>
            <run_depend>image_transport</run_depend>
            <run_depend>libopencv-dev</run_depend>
            <run_depend>sensor_msgs</run_depend>
            <run_depend>std_msgs</run_depend>
            ```

2. Edit *opencv_example.cpp* in your *src* folder
    1. `gedit ~/catkin_ws/src/turtlebot_houston/src/opencv_example.cpp`
    2. Replace the *Hello ROS* code with the following *OpenCV* code:

        ```c
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
        ```
    
    3. Save and exit

3. Build and run your new code:
    1. `catkin_make --directory ~/catkin_ws --pkg turtlebot_houston`
    2. `source ~/devel/setup.sh`
    3. `rosrun turtlebot_houston opencv_example`


## Additional Resources
[ROSCPP Tutorials](http://wiki.ros.org/roscpp_tutorials)
[Cv_Bridge Tutorials](http://wiki.ros.org/cv_bridge/Tutorials)

[Return to the main README page](/README.md)
