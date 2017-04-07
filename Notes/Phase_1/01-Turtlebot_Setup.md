Install ROS on Lenovo Thinkpad x260
====================================

Installing Ubuntu 16.04 
--------------------------------------
 Using the Turtlebot 16.04 USB stick
1. Insert USB stick into laptop
2. Power on laptop
3. Enable boot from USB:
    1) [Enter boot menu](https://support.lenovo.com/us/en/solutions/ht500222) by pressing <F1/F12> 
    2) Navigate to `Boot` menu pane
    3) Navigate to `Boot Device Priority`
    4) Set `1st Boot Device` to USB 
    5) Save new configuration by pressing 
4. Shut down machine
5. Insert Turtlebot 16.04 USB stick
6. Power on laptop
7. Follow GUI menu prompts to install Ubunut
8. Power off and remove installation media

Install ROS Kinetic
-------------------------------------------------------------------------------------------
Follow the instructions at http://wiki.ros.org/kinetic/Installation/Ubuntu:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall
```

Install Turtlebot packages
---------------------------
```bash
sudo apt-get install ros-kinetic-turtlebot* ros-kinetic-astra-*
```

(Optional) Install Turtlebot Branding
--------------------------------------
```bash
sudo apt install git
mkdir ~/tmp && cd ~/tmp
git clone https://github.com/TurtleBot-Mfg/turtlebot-doc-indigo
git clone https://github.com/TurtleBot-Mfg/turtlebot-env-indigo
git clone https://github.com/TurtleBot-Mfg/turtlebot-branding-indigo
git clone https://github.com/TurtleBot-Mfg/turtlebot-wallpapers
sudo cp -r ~/tmp/turtlebot-branding-indigo/root/lib/plymouth/themes /usr/share/plymouth/themes
sudo cp -r ~/tmp/turtlebot-branding-indigo/root/usr/share/themes /usr/share/plymouth/themes
sudo cp -r ~/tmp/turtlebot-doc-indigo/root/etc/skel/* /etc/skel/.
cp ~/tmp/turtlebot-doc-indigo/root/etc/skel/Desktop/turtlebot-doc.desktop ~/Desktop
sudo cp -r ~/tmp/turtlebot-doc-indigo/root/usr/share/doc/turtlebot /usr/share/doc/.
sudo cp -r ~/tmp/turtlebot-env-indigo/root/etc/* /etc/.
sudo cp -r ~/tmp/turtlebot-env-indigo/root/usr/share/glib-2.0/schemas /usr/share/glib-2.0/schemas/.
sudo /usr/bin/glib-compile-schemas /usr/share/glib-2.0/schemas/
sudo cp -r ~/tmp/turtlebot-wallpapers/root/usr/share/backgrounds/* /usr/share/backgrounds/.
```

