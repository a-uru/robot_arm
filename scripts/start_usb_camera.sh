#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
# USB camera uses Fast DDS even if a previous shell selected Zenoh.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Clear the transport overrides introduced during camera troubleshooting.
unset FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
camera_setup="$workspace_dir/install/camera/local_setup.bash"
if [[ ! -f "$camera_setup" ]]; then
  echo "Camera overlay is not built. See docs/usb_camera_source.md" >&2
  exit 1
fi
source "$camera_setup"
# Raw RGB frames exceed the default Fast DDS shared-memory segment.
export FASTRTPS_DEFAULT_PROFILES_FILE="$workspace_dir/config/camera/fastdds_camera.xml"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace_dir/log/usb_camera}"
mkdir -p "$ROS_LOG_DIR"
camera_device="${VIDEO_DEVICE:-/dev/v4l/by-id/usb-XWF-1080p6_Camera_XWF-1080p6_Camera_202403191545002-video-index0}"
if [[ ! -e "$camera_device" ]]; then
  echo "Camera device not found: $camera_device" >&2
  echo 'Set VIDEO_DEVICE=/dev/videoN to choose another camera.' >&2
  exit 1
fi
if command -v fuser >/dev/null 2>&1 && fuser -s "$camera_device"; then
  echo "Camera is already in use: $camera_device" >&2
  echo 'Stop the existing camera node with Ctrl+C before starting another.' >&2
  exit 1
fi
exec ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __node:=usb_camera \
  --params-file "$workspace_dir/config/camera/usb_camera.yaml" \
  -p "video_device:=$camera_device" "$@"
