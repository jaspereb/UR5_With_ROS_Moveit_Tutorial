# UR5_With_ROS_Moveit_Tutorial
A tutorial on setting up a UR CB arm to run with ROS and Moveit!

There are a few tricks to setting up a UR arm for use with ROS, after working through these I decided to document the process for other users. This tutorial is specifically how I set up a UR5 CB series arm with Moveit! although it should be a similar process for any other arm supported by the ur_modern_driver package. 

# Requirements
-To begin with you should have ROS kinetic installed by following http://wiki.ros.org/kinetic/Installation/Ubuntu 

-I tested this code with the UR polyscope version 3.9.1 but previous versions should also work. If you are updating the polyscope software you need to do each subversion in order (3.4.X, 3.5.X, 3.6.X and so on). You can get these files from the UR website, you will need a USB to copy them onto.

-You need a network connection to the arm, the simplest method is to set DHCP under the settings->network menu and read the IP address once it appears there. You can also run a direct ethernet connection using the 'Configure Your Hardware' instructions [here](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial) 

-Note the IP of the robot and check you can ping it using 
`ping IP_FROM_ABOVE`

# Installations
Follow only the installation portions of the instructions from https://github.com/ros-industrial/universal_robot and https://github.com/ros-industrial/ur_modern_driver/. If you also try to execute the demonstration commands the robot may not behave as you expect due to the issues described in the 'Known Problems' section.


-Download the Universal Robot repo from https://github.com/ros-industrial/universal_robot into your catkin workspace. Do not use the apt-get command but compile from source by following the 'Building from Source' instructions. **Important: Make sure you clone the kinetic devel branch using:**

`cd ~/catkin_ws/src
git clone -b kinetic-devel git@github.com:ros-industrial/universal_robot.git`

-Build your catkin workspace by running

`cd ~/catkin_ws
catkin_make clean
catkin_make install`

If you get any errors at this stage you will need to stop and fix them, these are typically due to missing dependencies which may be fixed by running

`rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src`

-Download the ur_modern_driver package kinect-devel branch into your catkin workspace using 

`cd ~/catkin_ws/src
git clone -b kinetic-devel git@github.com:ros-industrial/ur_modern_driver.git`

then rebuild your catkin workspace with 

`cd ~/catkin_ws
catkin_make clean
catkin_make install`

You now have all of the required packages installed.


CHANGE TO USE LOW BANDWIDTH CONTROLLER??

# Known Problems
There are three major issues with the default setup of the packages you installed:

1) The controllers are dependent on the speed setting on the UR tablet. At 100% speed the moveit motion commands and demonstration scripts (like the ur_driver test_move.py script) move extremely quickly. 

2) Moveit! fails to create reasonable plans without reduced joint angle limits.

3) The ur_modern_driver by default 


