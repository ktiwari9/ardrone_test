This instructions are for start working with the Parrot Ardrone 2.0, using ROS's driver ardrone_autonomy. We resume the instructions for installling ROS Kinetic and the drone's driver. Then a little explanation of how create a package (dependencies that are needed), compiling it, and where to create and link C++ files. We also cover how to handle images and navdata from the drone, and how to build a simple controller for the vehicule.


This document doesn't pretend to have complete information on how ROS's catkin_make build method work . For any further references visit wiki.ros.org, and the ardrone_autonomy documentation in http://ardrone-autonomy.readthedocs.io/.


###ROS Kinetic Installation
This version only works with Ubuntu Wily (15) and Xenial (16.04 LTS). And the complete guide is in http://wiki.ros.org/kinetic/Installation/Ubuntu .

First, add the ROS packages to sources.list:

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
Set up keys, for no middle man problems:
```sh
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
```
Update:
```sh
sudo apt-get update
```
Install the full version:
```sh
sudo apt-get install ros-kinetic-desktop-full
```
Since is the first time, run:
```sh
sudo rosdep init
rosdep update
```
To install package directly from command line (this allows you to run apt-get install ros-<packagename> in the default workspace) and also install catkin:
```sh
sudo apt-get install python-rosinstall
sudo apt-get install catkin
```
Now, we create a ros workspace, idealy in your home folder:
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
```
The  `catkin_make ` command build all the packages that are in the catkin_ws/src folder. We source `devel/setup.bash` to add our new workspace to $ROS_PACKAGE_PATH enviroment variable. We might want to have the configuration to be charge always we open a new bash session:
```sh
echo "source /home/<username>/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
We also might have permission troubles with directories. For not hav any issue, give to your user all permission over /.ros directory:
```sh
cd /home/<username>/
sudo chown -R <username>:<username> .ros
```
Thus, we are ready to install any package and make all ROS tutorials (http://wiki.ros.org/ROS/Tutorials).


###ardrone_autonomy driver installation 

First, we go to our working space and then clone the git repository. Then we make sure that all dependencys for the work space are installed. And build.
```sh
cd ~/catkin_ws/src
git clone https://github.com/AutonomyLab/ardrone_autonomy.git
cd ..
rosdep install --from-paths src -i
catkin_make
```
If the error 'ROS distro is not set' appear, you should add rosdistro option:
```sh
rosdep install --from-paths src -i --rosdistro=kinetic
```
The build process should end with no errors. If there is any, you should verify if you are working in the correct workspace:
```sh
echo $ROS_PACKAGE_PATH
```
Yours should appear in the list, if not, you can source the `/devel/setup.bash`.
