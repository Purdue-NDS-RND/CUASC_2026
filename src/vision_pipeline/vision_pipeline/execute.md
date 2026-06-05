# **System Architecture & Launch Instructions**

To run the full end-to-end vision and geolocation pipeline on the physical drone, you will need to open five separate SSH terminals to the Jetson companion computer. **Note:** Ensure you have sourced the ROS 2 workspace (source /opt/ros/humble/setup.bash && source ~/dev/CUASC_2026/install/setup.bash) in every terminal before running these commands.



## **Terminal 1: Hardware Telemetry Bridge (MAVROS)**

Description: This command establishes the vital communication bridge between the Jetson companion computer and the Pixhawk flight controller. It translates native ArduPilot MAVLink messages into standardized ROS 2 topics. This is required to get real-time GPS coordinates and drone orientation (IMU) data into the vision pipeline.

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM1:921600
```

**Settings & Parameters:**
- ```apm.launch```: Instructs MAVROS to use the ArduPilot dialect.
- ```fcu_url:=/dev/ttyACM1:921600```: Tells the Jetson to look for the flight controller on USB port ttyACM1 and communicate at a baud rate of 921600 (the standard high-speed USB configuration for Pixhawk telemetry).


## **Terminal 2: Ground Station Telemetry (Foxglove)**

Description: This runs a lightweight WebSocket server that streams live ROS 2 data over the Wi-Fi network. It allows a remote laptop running Foxglove Studio to view live 4K camera feeds, 3D TF frames, and plotting data while the drone is in flight, without needing a physical monitor attached to the Jetson.


```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

**Settings & Parameters:**
- ```port:=8765```: Forces the WebSocket to bind to port 8765. To view the data, the remote ground station must connect to ws://<JETSON_IP>:8765.

# **Terminal 3: Updated (Combines 3-5)** 

(This single command now safely launches the URDF Publisher, the Camera, YOLO, and the Logger, reading all offsets and headless settings directly from the YAML).
```bash
cd ~/dev/CUASC_2026/
./set_stream_rate.sh
cd ~/dev/CUASC_2026/src/vision_pipeline/vision_pipeline
./vision.sh
```


<!--
# **Terminal 3: Physical Geometry Broadcaster (URDF)**

Description: This node reads the physical layout of the drone from the hexacopter.urdf file and broadcasts it to the ROS 2 Transform (TF2) tree. It explicitly defines where the camera (camera_optical_frame) is physically bolted relative to the flight controller (base_link), which is necessary for accurate 3D visualization and spatial math.

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/dev/CUASC_2026/src/vision_pipeline/urdf/hexacopter.urdf)"
```

**Settings & Parameters:**
- ```robot_description```: Parses the raw XML text from the URDF file and injects it into the ROS 2 parameter server so all other nodes can reference the drone's physical structure

# **Terminal 4: Core Vision Pipeline (Camera & AI)**

Description: This launch file acts as the conductor for the two heaviest processes in the system. It boots up the image_grabber and yolo_node.

1. **Image Grabber:** Connects to the Arducam via a GStreamer pipeline, instantly rectifies lens distortion using pre-computed calibration matrices, and publishes a perfectly flat image to /camera/image_raw.
2. **YOLO Node:** Slices the 4K image, runs hardware-accelerated TensorRT inference (yolo26n_v2.1.engine), applies Non-Maximum Suppression, and publishes an array of bounding boxes to /drone_control/detection.

```bash
ros2 launch vision_pipeline vision_demo.launch.py
```

# **Terminal 5: The Mission Logger (Black Box & Geolocator)**

Description: This is the master recording and calculation node. It runs two completely independent asynchronous tasks:
1. **Continuous Flight Recorder:** It listens directly to the camera feed and saves every single frame to the Jetson's SSD (bypassing YOLO completely to ensure no dropped frames).
2. **Target Geolocator:** It uses a message_filter to perfectly synchronize YOLO bounding boxes with the corresponding image. It then pulls the drone's IMU data from MAVROS, calculates a mathematical raycast from the pixel to the ground, draws a red target circle on the frame, and logs the geographic (Lat/Lon) hit to mission_log.csv.

```bash
ros2 run vision_pipeline mission_logger --ros-args -p mount_x:=-0.127 -p mount_y:=0.0 -p mount_z:=-0.1524 -p show_debug_window:=false
```

**Settings & Parameters:**
- ```mount_x, mount_y, mount_z```: Hardcodes the measured physical offset of the camera lens (in meters) relative to the flight controller in the NED (North-East-Down) body frame. This ensures the raycast math relies on ground-truth measurements rather than standard ROS ENU assumptions.
- ```show_debug_window:=false```: Runs the node in strict "headless" mode. This disables OpenCV from attempting to render a live GUI pop-up window, which prevents the node from crashing when the drone is flying without an HDMI monitor plugged in.-->
