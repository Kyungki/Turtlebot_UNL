#!/bin/bash
# Write our program to bash alias
DABIT_DIR=$(dirname $(readlink -f $0))
cp "$DABIT_DIR/../.bash_aliases" ~/.dabit_aliases
cp ~/.bashrc ~/.bashrc_backup

echo $DABIT_DIR
sed -i '1s!^!export DABIT_DIR='"$DABIT_DIR"'\n!' ~/.dabit_aliases

if [ -f ~/.dabit_aliases ]; then
    . ~/.dabit_aliases
else
    echo "Something went wrong, aborting"
    return 0
fi

function remove_call(){
  echo "sed 's/$1/#&/g' $2"
  sed 's/$1/#&/g' $2
}

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
" >> ~/.bashrc

. ~/.bashrc
dabit-setup-utility test
