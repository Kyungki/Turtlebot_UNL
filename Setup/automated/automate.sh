#!/bin/bash
# Write our program to bash alias
if [ -z "$DABIT_DIR" ]; then
    DABIT_DIR=$(dirname $(readlink -f $0))"/../.."
fi
cp "$DABIT_DIR/Setup/.bash_aliases" ~/.dabit_aliases
cp "$DABIT_DIR/Setup/.rosrc" ~/.rosrc
cp ~/.bashrc ~/.bashrc_backup

sed -i '1s!^!export DABIT_DIR='"$DABIT_DIR"'\n!' ~/.dabit_aliases

. ~/.dabit_aliases

function remove_call(){
  echo "sed 's/$1/#&/g' $2"
  sed 's/$1/#&/g' $2
}

sed -i -e '/-f ~\/.dabit_aliases/,+15d' ~/.bashrc
sed -i "s/^source \/opt\/ros\/kinetic\/setup.bash/#&/g" ~/.bashrc
sed -i "s/^export TURTLEBOT_BASE=/#&/g" ~/.bashrc
sed -i "s/^export TURTLEBOT_3D_SENSOR=/#&/g" ~/.bashrc
sed -i "s/^export TURTLEBOT_STACK=/#&/g" ~/.bashrc
sed -i "s/^export ROS_MASTER_URI=/#&/g" ~/.bashrc
sed -i "s/^export ROS_IP=/#&/g" ~/.bashrc
sed -i "s/^export ROS_HOSTNAME=/#&/g" ~/.bashrc
sed -i "s/^export ROS_HOME=/#&/g" ~/.bashrc

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
