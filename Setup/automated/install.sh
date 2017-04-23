#!/bin/bash

###
# Define definitions
###

. "$DABIT_DIR/Setup/automated/definitions.sh"

###
# Backup Files
###

. "$_dabit_dir/Setup/automated/backup.sh"

###
# Workspace Setup
###

__setup_workspace(){
  # Set up new workspace
  mkdir -p "$_catkin_ws_dir"
  mkdir -p "$_workspace_dir"
  mkdir -p "$_tmp_dir"
}
echo "Setting up new workspace"
__setup_workspace

###
# Catkin Setup
###

__make_catkin_workspace(){
  # Set up catkin workspace
  mkdir -p "$_catkin_ws_dir/src"
  catkin_make --directory "$_catkin_ws_dir"
}

echo "Making new catkin workspace"
__make_catkin_workspace

###
# Copy Packages
###

__copy_catkin_ws(){
  for i in "${_dabit_packages[@]}"
  do
    cp -rv "$_dabit_dir/Setup/catkin_ws/src/$i" "$_catkin_ws_dir/src"
  done
}

echo "Copying ROS packages"
__copy_catkin_ws

###
# Install Dependencies
###

__install_dependencies(){
  echo "install dependencies"
}

echo "Installing dependencies"
__install_dependencies

###
# Build Packages
###

__build_ws(){
  echo "build workspace"
}

echo "Building Workspace"
__build_ws


###
# Running Test Code
###

__run_checks(){
  echo "run checks"
}

echo "Running Checks and Tests"
__run_checks


echo "Installation Complete"
