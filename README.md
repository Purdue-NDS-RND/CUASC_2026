Setting up ROS Humble With Simulation
=================================================================================

General Note: any installation instructions online that use a dir that ends in "_ws" should now use VTOL_ws instead.

ROS2 Humble Installation
-----------------------
Go to this link for refrence. Please install the ros-humble-desktop
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html 

To be able to use ROS2 commands in terminal, you need to source the setup file.
You could type a command every time you open a terminal or you could add it to your .bashrc file. After running the command, open up bashrc to make sure it's added at the very bottom.
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


Testing ROS2 Installation
-----------------------
To test if ROS2 is installed correctly, open a new terminal and run the following command:
ros2 run demo_nodes_cpp talker  
If you see output similar to the following, then ROS2 is installed correctly:
[INFO] [talker]: Publishing: 'Hello, world! 0'
[INFO] [talker]: Publishing: 'Hello, world! 1'
...
In another terminal, run the following command:
ros2 run demo_nodes_cpp listener
If you see output similar to the following, then ROS2 is installed correctly:
[INFO] [listener]: I heard: 'Hello, world! 0'
[INFO] [listener]: I heard: 'Hello, world! 1'
... 

ROS Build Tools Installation
-----------------------
To build ROS2 packages from source, you need to install some build tools. You can install them by running the following command:
sudo apt install -y build-essential cmake git python3-colcon-common-extensions python3-pip python3-rosdep python3-vcstool python3-catkin-tools


Gazebo Harmonic Installation
-----------------------
The following link is used for reference:
https://gazebosim.org/docs/harmonic/install_ubuntu/

To install Gazebo Harmonic according to the instructions on the page.

Install MAVROS
-----------------------
https://docs.ros.org/en/humble/p/mavros/

Commands:
sudo apt install ros-humble-mavros ros-humble-mavros-extras

cd ~/Downloads
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh


