# **Arducam Calibration & Rectification Pipeline (ROS 2 Humble)**

This guide details the complete workflow for intrinsically calibrating the 4K Arducam on the NVIDIA Jetson, integrating the mathematical matrices into our ROS 2 pipeline, and flattening (rectifying) the raw image feed for downstream YOLO object detection.

## **🛠 Prerequisites**

* **Hardware:** NVIDIA Jetson (with attached monitor/display), Arducam IMX519.  
* **Calibration Target:** 6x8 interior corner checkerboard, 0.025m (25mm) squares.  
* **Packages:** Ensure ROS 2 calibration and processing tools are installed:  
```bash
  sudo apt update  
  sudo apt install ros-humble-camera-calibration ros-humble-image-proc
```
## **🛑 Troubleshooting: Hardware Locks**

If you ever start the image_grabber node and see a stream of Failed to capture image from camera! alongside a Failed to create CaptureSession error, the Jetson's camera daemon has locked up. **Do not reboot the Jetson.**  
Simply restart the background daemon: 
```bash
sudo systemctl restart nvargus-daemon
```
Wait 3 seconds, and start your node again.

## **Step 1: Live Camera Calibration**

To get an accurate mapping of the lens distortion, we must feed the ROS 2 solver a highly diverse set of perspectives using the checkerboard.  
**1. Start the camera node:**  
```bash
ros2 run vision_pipeline image_grabber
```

**2. Start the calibration GUI (in a new terminal):**  
*Note: The --no-service-check flag MUST be placed before --ros-args, otherwise ROS 2's argument parser will crash.*  
```bash
ros2 run camera_calibration cameracalibrator --size 6x8 --square 0.025 --no-service-check --ros-args -r "image:=/camera/image_raw" -r "camera:=/camera"
```

**3. The Calibration "Dance":**  
To turn the progress bars (X, Y, Size, Skew) green, perform these specific movements:

* **X / Y (Radial Distortion):** Move the board to the extreme left, right, top, and bottom edges of the camera frame. This maps the "fisheye" warping at the edges of the glass.  
* **Size (Focal Length):** Hold the board extremely close to the lens (filling the frame), then slowly pull it as far back as your arm can reach.  
* **Skew (Tangential Distortion):** Pitch the board forward/backward 45 degrees, yaw it left/right, and spin it.

Once all bars are green and you have enough samples, click **CALIBRATE**. The Jetson will freeze for a few minutes to crunch the 4K matrices. Once finished, click **SAVE**.

## **Step 2: Extracting the Matrices**

When you click "Save", the GUI dumps a compressed archive into your temporary folder.  
**1. Extract the archive:**  
```bash
cd /tmp  
tar -xvf calibrationdata.tar.gz
```

**2. Move and rename the YAML file to our package:**  
```bash
mkdir -p ~/dev/CUASC_2026/src/vision_pipeline/config  
mv /tmp/ost.yaml ~/dev/CUASC_2026/src/vision_pipeline/config/arducam_info.yaml
```

## **Step 3: Code Integration**

For ROS 2 to use these matrices, our image_grabber node must publish them on the /camera/camera_info topic in sync with the raw images.  
**1. Update setup.py**  
To ensure the build system copies our new config folder, add these imports and modify the data_files array in ~/dev/CUASC_2026/src/vision_pipeline/setup.py:  
```python
import os  
from glob import glob  
from setuptools import find_packages, setup

# ... inside setup() ...  
data_files=[  
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  
    ('share/' + package_name, ['package.xml']),  
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  
],
```

**2. Rebuild the package:**  
```bash
cd ~/dev/CUASC_2026  
colcon build --packages-select vision_pipeline  
source install/setup.bash
```

## **Step 4: Rectification (Fixing the "Smear of Death")**

We use ROS 2's image_proc to mathematically flatten the image.  
**⚠️ CRITICAL NOTE:** Do not run the standard image_proc container. Because the Jetson's hardware ISP already debayers and colors our image (bgr8), the standard container will try to re-color it, resulting in a black-to-white gradient known as the "Smear of Death".  
Instead, run *only* the rectify_node directly to bypass the color-scrambler:  
```bash
ros2 run image_proc rectify_node --ros-args -r image:=/camera/image_raw -r camera_info:=/camera/camera_info -r image_rect:=/camera/image_rect_color
```

## **Step 5: Verification**

To prove the math is working and the image is geometrically flat:  
**1. Open the visualizer:**  
```bash
ros2 run rqt_image_view rqt_image_view
```

**2. The Doorframe Test:**

* Point the camera so a straight vertical line (like a doorframe or wall corner) is at the extreme left or right edge of the screen.  
* Select /camera/image_raw in the dropdown. You will see the edge bowing outward ( ).  
* Select /camera/image_rect_color. The edge should instantly snap mathematically straight | |.

The feed is now officially ready to be piped into YOLO for accurate GPS raycasting!
