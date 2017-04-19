## Fiducial Markers


## PREQUISITES


## Installation
1. Install OpenCV libraries
    * `sudo apt install libopencv*`
2. Download, the apriltags-cpp opencv library
    * `cd ~/workspace`
    * `git clone -b kinetic-devel https://github.com/locusrobotics/apriltags_cpp`
    * `mkdir apriltags_cpp/build`
    * `cd apriltags_cpp/build`
    * `cmake ..`
    * `make`
    * `sudo make install`
3. Build apriltags-cpp in as an isolated package
    * `cd ~/catkin_ws`
    * `catkin_make_isolated --pkgname apriltagscpp`
2. Download the apriltags library
    * `git clone -b kinetic-devel https://github.com/locusrobotics/apriltags`
3. `cd ~/catkin_ws`
    * `catkin_make`

## 
