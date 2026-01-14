# Robot Arm Development

## Overview

This is a personal research project focused on robot intelligence using open-source robot arm platforms.

**Current Status:**
- ✅ PS4 controller teleoperation implemented (October 30, 2025)
- 🔄 Progress will be slower until June 2026 due to school robotics commitments

**Key Features:**
- ROS2-based robot arm control system
- Teleoperation with PS4 controller
- Camera integration for visual feedback
- RViz visualization support

---

## Table of Contents

- [Teleoperation Setup](#teleoperation-setup)
- [Camera Setup](#camera-setup)
- [Troubleshooting](#troubleshooting)

---

## Teleoperation Setup

Follow these steps to operate the robot arm with a PS4 controller:

### Step 1: Connect PS4 Controller
Start the joy node to interface with the PS4 controller:
```bash
ros2 run joy joy_node
```

### Step 2: Connect to Hardware
⚠️ **Warning:** The robot arm will move to its initial position upon connection.

```bash
cd ~/robot_arm
source install/setup.bash
ros2 run so101_hw_interface so101_motor_bridge
```

### Step 3: Launch RViz Visualization
Visualize the robot arm state in RViz:
```bash
cd ~/robot_arm
source install/setup.bash
ros2 launch so101_follower_description display.launch.py \
    use_gui:=false \
    joint_states_topic:=/so101_follower/joint_states
```

### Step 4: Start Control Node
Launch the control interface:
```bash
cd ~/robot_arm/Lerobot_ros2/src/so101_hw_interface/controll/
python3 keyboard_controller.py
```

---

## Camera Setup

### List Available Camera Devices

Check connected cameras on your system:
```bash
v4l2-ctl --list-devices
```

**Available cameras on this system:**
- **Integrated Camera** (built-in): `/dev/video0`, `/dev/video1`
- **XWF-1080p6 Camera** (external): `/dev/video2`, `/dev/video3`

### Launch Camera Node

**For external camera (XWF-1080p6):**
```bash
cd ~/robot_arm
source install/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video2
```

**For built-in camera:**
```bash
cd ~/robot_arm
source install/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

### View Camera Feed

After launching the camera node, use one of the following methods to view the feed:

#### Method 1: Using rqt_image_view
```bash
cd ~/robot_arm
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```
Select `/image_raw` from the dropdown menu in the top-left corner.

#### Method 2: Using RViz
After launching RViz:
1. Click the **"Add"** button in the bottom-left
2. Select **"By display type"** → **"Image"**
3. Set the **"Image Topic"** to `/image_raw`

### Camera Topics

- `/image_raw` - Camera image (sensor_msgs/msg/Image)
  - Resolution: 640x480
  - Encoding: rgb8
- `/camera_info` - Camera calibration info (sensor_msgs/msg/CameraInfo)

---

## Troubleshooting

### Camera Feed Not Displaying

Check if topics are being published:
```bash
# List image-related topics
ros2 topic list | grep image

# Check publishing frequency
ros2 topic hz /image_raw

# View a single message
ros2 topic echo /image_raw --once
```

### Common Issues

- **No video device found:** Check device permissions and ensure the camera is properly connected
- **Permission denied:** Run `sudo chmod 666 /dev/video*` or add your user to the `video` group
- **Low frame rate:** Reduce image resolution or check USB bandwidth

---

## License

This project is open source. Please check individual package licenses for details.
