#!/bin/bash

###
# Define definitions
###

# Directory names
_backup="backup"
_catkin_ws="catkin_ws"
_workspace="workspace"
_tmp="tmp"
_dabit_dir="$DABIT_DIR"

# File Names
_rosrc=".rosrc"
_aliases=".dabit_aliases"
_bashrc=".bashrc"

# Where above directories and files are all located
_user_dir="$HOME"
_backup_dir="$_user_dir/$_backup"
_catkin_ws_dir="$_user_dir/$_catkin_ws"
_workspace_dir="$_user_dir/$_workspace"
_tmp_dir="$_user_dir/$_tmp"
_arduino_dir="$_user_dir/Arduino"
_pwd=$(pwd)

_rosrc_dir="$_user_dir/$_rosrc"
_aliases_dir="$_user_dir/$_aliases"
_bashrc_dir="$_user_dir/$_bashrc"

# Which Dabit Packages to install
declare -a _ros_packages=("apriltags" "turtlebot_houston" "rqt_turtlebot_houston" "turtlebot_wanderer")
declare -a _workspace_packages=("apriltags_cpp")
declare -a _workspace_install_packages=("apriltags_cpp")

# Miscelleneous Defines
_arduino_tar="arduino-1.8.2-linux64.tar.xz"
