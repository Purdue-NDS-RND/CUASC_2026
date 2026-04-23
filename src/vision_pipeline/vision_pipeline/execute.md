# **System Architecture & Launch Instructions**

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

# **Terminal 3** 

(This single command now safely launches the URDF Publisher, the Camera, YOLO, and the Logger, reading all offsets and headless settings directly from the YAML).
```bash
./vision.sh
```
