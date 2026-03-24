# 🚁 Setting up ROS Humble With Simulation

> **Note:** Any installation instructions online that use a directory ending in `_ws` should now use `VTOL_ws` instead.

---

## 📦 Installation

### 1. ROS 2 Humble

**Reference:** https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
> Install the `ros-humble-desktop` variant.

Source ROS 2 in every terminal by adding it to your `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

### 2. Testing ROS 2 Installation

Open two terminals and verify communication:

**Terminal 1 — Talker:**
```bash
ros2 run demo_nodes_cpp talker
```
Expected output:
```
[INFO] [talker]: Publishing: 'Hello, world! 0'
[INFO] [talker]: Publishing: 'Hello, world! 1'
...
```

**Terminal 2 — Listener:**
```bash
ros2 run demo_nodes_cpp listener
```
Expected output:
```
[INFO] [listener]: I heard: 'Hello, world! 0'
[INFO] [listener]: I heard: 'Hello, world! 1'
...
```

---

### 3. ROS Build Tools

```bash
sudo apt install -y build-essential cmake git \
  python3-colcon-common-extensions python3-pip \
  python3-rosdep python3-vcstool python3-catkin-tools
```

---

### 4. Gazebo Harmonic

**Reference:** https://gazebosim.org/docs/harmonic/install_ubuntu/

Follow the instructions on the page to install Gazebo Harmonic.

---

### 5. ArduPilot SITL

**Reference:** *(link TBD)*

---

### 6. MAVROS

**Reference:** https://docs.ros.org/en/humble/p/mavros/

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras

cd ~/Downloads
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

---

## 🚀 Boot Simulation

```bash
# Launch simulation
ros2 launch ardupilot_gz_bringup iris_runway.launch.py

# Connect MAVROS (in a new terminal)
ros2 launch mavros apm.launch.py fcu_url:=udp://:14550@
```

---

## 🎯 Drone Control — Target Spawner & Follower

The `drone_control` package provides two nodes:

| Node | Description |
|------|-------------|
| `target_spawner` | Spawns a random target near the takeoff point, publishes its position, and respawns after hover |
| `target_follower` | Follows the target by publishing local position setpoints; handles GUIDED mode + arming |

### Topics

**Subscribed (MAVROS):**
| Topic | Type |
|-------|------|
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` |
| `/mavros/state` | `mavros_msgs/State` |
| `/mavros/home_position/home` | `mavros_msgs/HomePosition` |

**Published:**
| Topic | Type |
|-------|------|
| `/drone_control/target/pose` | `geometry_msgs/PoseStamped` |
| `/drone_control/target/gps` | `sensor_msgs/NavSatFix` |
| `/drone_control/target/status` | `std_msgs/String` |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` |

**Services Used:**
| Service | Type |
|---------|------|
| `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` |
| `/mavros/set_mode` | `mavros_msgs/srv/SetMode` |
| `/world/<world>/create` | `ros_gz_interfaces/srv/SpawnEntity` |

### Notes

- Target positions are in MAVROS local ENU. GPS output is an approximate conversion using the home position.
- Spawn uses `allow_renaming=true` so it won't fail if a box with the same name already exists.

### Run

```bash
# 1. Build
colcon build --packages-select drone_control
source install/setup.bash

# 2. Launch
ros2 launch <package> <launch_file>
# Example:
ros2 launch drone_control waypoint_demo.launch.py
```

---

## ✈️ Simple Takeoff (no targets)

Handles mode switching, arming, and takeoff via MAVROS. **Only takes off when explicitly requested via service call.**

> **Prerequisites:** Must have ArduPilot SITL + Gazebo + MAVROS running first

### Service-Based Manual Takeoff

Start the node and trigger takeoff when ready:

```bash
# Terminal 1: Start Gazebo + ArduPilot SITL + MAVROS
ros2 launch ardupilot_gz_bringup iris_runway.launch.py

# Terminal 2: Run the takeoff node
ros2 run drone_control simple_takeoff

# Terminal 3: Trigger takeoff when ready
ros2 service call /drone_control/takeoff mavros_msgs/srv/CommandTOL \
  "{altitude: 25.0, min_pitch: 0.0, yaw: 0.0}"
```

### Auto Takeoff (Optional)

To enable automatic takeoff on startup:

```bash
ros2 run drone_control simple_takeoff --ros-args -p auto_takeoff:=true
```

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `takeoff_altitude_m` | `20.0` | Default takeoff altitude |
| `auto_takeoff` | `true` | Takeoff automatically on startup |
| `arm_on_start` | `true` | Arm automatically |
| `set_guided_mode` | `true` | Switch to guided mode |
| `guided_mode_name` | `"GUIDED"` | Flight mode name |
| `max_arm_attempts` | `5` | Max arming retries |

---

## ⚙️ Bashrc Recommendations

Add the following to `~/.bashrc` to avoid re-sourcing every session:

```bash
# ROS 2
source /opt/ros/humble/setup.bash
source ~/Programming/VTOL_ws/install/setup.bash
alias build='colcon build && source install/setup.bash'

# Gazebo
export GZ_VERSION=harmonic
export PATH=$PATH:/home/ppatel/Programming/VTOL_ws/Micro-XRCE-DDS-Gen/scripts
```
