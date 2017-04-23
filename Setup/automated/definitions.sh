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

_rosrc_dir="$_user_dir/$_rosrc"
_aliases_dir="$_user_dir/$_aliases"
_bashrc_dir="$_user_dir/$_bashrc"

# Which Dabit Packages to install
declare -a _dabit_packages=("apriltags" "turtlebot_houston" "turtlebot-rqt-slam" "turtlebot_wanderer")

