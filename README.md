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


Ardupilot Sitl Installation
-----------------------
The following link is used for reference:



Install MAVROS
-----------------------
https://docs.ros.org/en/humble/p/mavros/

Commands:
sudo apt install ros-humble-mavros ros-humble-mavros-extras

cd ~/Downloads
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh



Drone Control (Target Spawner + Follower)
----------------------------------------
This workspace includes a Python ROS 2 package `drone_control` with two nodes:

- `target_spawner`: spawns a random target near the takeoff point, publishes its position, and respawns after hover.
- `target_follower`: follows the target by publishing local position setpoints and handles GUIDED + arming.

Topics
- Subscribed (MAVROS):
  - `/mavros/local_position/pose` (geometry_msgs/PoseStamped)
  - `/mavros/state` (mavros_msgs/State)
  - `/mavros/home_position/home` (mavros_msgs/HomePosition)
- Published:
  - `/drone_control/target/pose` (geometry_msgs/PoseStamped)
  - `/drone_control/target/gps` (sensor_msgs/NavSatFix)
  - `/drone_control/target/status` (std_msgs/String)
  - `/mavros/setpoint_position/local` (geometry_msgs/PoseStamped)
- Services used:
  - `/mavros/cmd/arming` (mavros_msgs/srv/CommandBool)
  - `/mavros/set_mode` (mavros_msgs/srv/SetMode)
  - `/world/<world>/create` (ros_gz_interfaces/srv/SpawnEntity)

Notes
- Target positions are in MAVROS local ENU. GPS output is an approximate conversion using the home position.
- Spawn uses `allow_renaming=true` so it won’t fail if a box with the same name already exists.

Run
1) Build:
   - `colcon build --packages-select drone_control`
   - `source install/setup.bash`
2) Launch:
   - `ros2 launch drone_control target_demo.launch.py`

Simple takeoff (no targets)
---------------------------
Runs a minimal node that switches to GUIDED, arms, and climbs to 20m above the current position.

- `ros2 run drone_control simple_takeoff`

Useful params:
- `takeoff_altitude_m` (default `20.0`)
- `arm_on_start` (default `true`)

Useful launch params
- `world_name` (default `iris_runway`)
- `spawn_service` (default `/world/map/create`)
- `radius_m`, `hover_radius_m`, `hover_duration_s`, `target_altitude_m`

Example:
- `ros2 launch drone_control target_demo.launch.py radius_m:=50.0 target_altitude_m:=25.0`
