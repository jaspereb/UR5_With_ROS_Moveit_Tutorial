# UR5_With_ROS_Moveit_Tutorial
A tutorial on setting up a UR CB arm to run with ROS and Moveit!

There are a few tricks to setting up a UR arm for use with ROS, after working through these I decided to document the process for other users. This tutorial is specifically how I set up a UR5 CB series arm with Moveit! although it should be a similar process for any other arm supported by the ur_modern_driver package. 

# Requirements
-To begin with you should have ROS kinetic installed by following http://wiki.ros.org/kinetic/Installation/Ubuntu 

-I tested this code with the UR polyscope version 3.9.1 but previous versions should also work. If you are updating the polyscope software you need to do each subversion in order (3.4.X, 3.5.X, 3.6.X and so on)

-You need a network connection to the arm, the simplest method is to set DHCP under the settings->network menu and read the IP address once it appears there. You can also run a direct ethernet connection using the 'Configure Your Hardware' instructions [here](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial) 

-Note the IP of the robot and check you can ping it using 
`ping IP_FROM_ABOVE`

-Download the Universal Robot repo from https://github.com/ros-industrial/universal_robot. **Important: Make sure you clone the kinetic devel branch using:**

`cd ~/catkin_ws/src
git clone -b kinetic-devel git@github.com:ros-industrial/universal_robot.git`

