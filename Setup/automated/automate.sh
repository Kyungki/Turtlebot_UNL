#!/bin/bash
# Write our program to bash alias
if [ -z "$DABIT_DIR" ]; then
    #DABIT_DIR=$(dirname $(readlink -f $0))"/../.."
    DABIT_DIR="$HOME/turtlebot-houston"
fi
cp "$DABIT_DIR/Setup/.dabit_aliases" ~/.dabit_aliases
cp "$DABIT_DIR/Setup/.rosrc" ~/.rosrc
cp ~/.bashrc ~/.bashrc_backup

sed -i '1s!^!export DABIT_DIR='"$DABIT_DIR"'\n!' ~/.dabit_aliases

. ~/.dabit_aliases

remove_call(){
  sed -i 's/'"$1"'/#&/g' "$2"
}

sed -i -e '/-f ~\/.dabit_aliases/,+15d' ~/.bashrc
remove_call "source \/opt\/ros\/kinetic\/setup.bash" ~/.bashrc
remove_call "export TURTLEBOT_BASE=" ~/.bashrc
remove_call "export TURTLEBOT_3D_SENSOR=" ~/.bashrc
remove_call "export TURTLEBOT_STACK=" ~/.bashrc
remove_call "export ROS_MASTER_URI=" ~/.bashrc
remove_call "export ROS_IP=" ~/.bashrc
remove_call "export ROS_HOSTNAME=" ~/.bashrc
remove_call "export ROS_HOME=" ~/.bashrc

echo "
if [ -f ~/.dabit_aliases ]; then
    . ~/.dabit_aliases
fi

if [ -f ~/.rosrc ]; then 
    . ~/.rosrc
fi

if [ -f /opt/ros/kinetic/setup.bash ]; then 
    . /opt/ros/kinetic/setup.bash
fi

if [ -f ~/catkin_ws/devel/setup.bash ]; then 
    . ~/catkin_ws/devel/setup.bash
fi
" >> ~/.bashrc

. ~/.bashrc
dabit-setup-utility test
