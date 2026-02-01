# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a web-based ROS point cloud visualization and interaction tool for UAV contact-based path planning. The project supports **both ROS 1 and ROS 2** with identical functionality but different backend implementations.

**Key Architecture:**
- **Frontend**: Pure JavaScript (Three.js + roslib.js) - identical for both ROS versions
- **Backend**: ROS with rosbridge_suite for WebSocket communication
- **Communication**: rosbridge protocol abstracts ROS 1/2 differences, allowing the same frontend to work with both

## Dual Version Structure

```
contact_webUI/
├── [ROS 1 Version - Root Directory]
│   ├── js/main.js, pointcloud.js, roi_selector.js
│   ├── launch/webui.launch (XML format)
│   ├── test/test_publisher.py (rospy)
│   └── start.sh
│
└── ros2_version/ [ROS 2 Version]
    ├── js/main.js, pointcloud.js, roi_selector.js (identical to ROS 1)
    ├── launch/webui.launch.py (Python format)
    ├── test/test_publisher.py (rclpy)
    └── start_ros2.sh
```

**Critical**: When modifying frontend JavaScript code, changes must be applied to BOTH versions since the files are duplicated.

## Common Commands

### One-Click Startup (Recommended)

```bash
# Start all services (rosbridge + PCD publisher + web server)
./start_all.sh

# Stop all services
./stop.sh

# Check service status
./check.sh
```

After startup, access the web UI at: **http://localhost:8000**

### ROS 1 Version

```bash
# Start rosbridge server
./start.sh
# OR
roslaunch contact_webUI/launch/webui.launch

# Start test data publisher (synthetic wavy plane)
python test/test_publisher.py

# Start PCD file publisher (uses test/test.pcd)
python test/test_pcd_publisher.py

# Start web server
python3 -m http.server 8000

# Verify topics
rostopic list
rostopic echo /cloud_in
rostopic echo /planning/roi_box
rostopic echo /planning/goal_points
rostopic hz /cloud_in

# Check rosbridge
rosnode list | grep rosbridge
```

### ROS 2 Version

```bash
# Start rosbridge server
cd ros2_version && ./start_ros2.sh
# OR
ros2 launch ros2_version/launch/webui.launch.py

# Start test data publisher
python3 ros2_version/test/test_publisher.py

# Start web server
cd ros2_version && python3 -m http.server 8000

# Verify topics
ros2 topic list
ros2 topic echo /cloud_in
ros2 topic echo /planning/roi_box
ros2 topic echo /planning/goal_points
ros2 topic hz /cloud_in

# Check rosbridge
ros2 node list | grep rosbridge
```

## Architecture Details

### Frontend Architecture (js/)

**main.js** (~845 lines) - Main application controller
- `ROSPointCloudApp` class orchestrates all components
- Manages ROS connection via roslib.js WebSocket
- Handles UI event binding and state management
- Coordinates between PointCloudManager and ROISelector
- Implements raycasting for point picking on point cloud surface
- Publishes to `/planning/roi_box` (PoseStamped) and `/planning/goal_points` (PoseArray)

**pointcloud.js** (~256 lines) - Point cloud rendering
- `PointCloudManager` class handles PointCloud2 messages
- Subscribes to `/cloud_in` (sensor_msgs/PointCloud2)
- Parses binary point cloud data from ROS messages
- Implements downsampling (maxPoints: 500,000) for performance
- Throttles updates (updateInterval: 100ms, throttle_rate: 200ms)
- Uses Three.js Points with BufferGeometry for efficient rendering

**roi_selector.js** (~150 lines) - ROI box manipulation
- `ROISelector` class manages 3D bounding box
- Creates semi-transparent green box (default: 2x2x2 meters)
- Integrates with Three.js TransformControls for translate/rotate/scale
- Provides box pose (position + quaternion) for ROS publishing

### ROS Communication Layer

**rosbridge_suite** provides WebSocket bridge:
- Default port: 9090
- Protocol: JSON over WebSocket
- Message types automatically converted between ROS 1/2
- Frontend uses roslib.js to communicate with rosbridge

**Subscribed Topics:**
- `/cloud_in` - sensor_msgs/PointCloud2 (global point cloud)
- `/mavros/local_position/pose` - geometry_msgs/PoseStamped (drone pose)

**Published Topics:**
- `/planning/roi_box` - geometry_msgs/PoseStamped (ROI bounding box pose)
- `/planning/goal_points` - geometry_msgs/PoseArray (start and end points)

### Key Differences Between ROS 1 and ROS 2 Versions

**Launch Files:**
- ROS 1: XML format (`webui.launch`)
- ROS 2: Python format (`webui.launch.py` with `generate_launch_description()`)

**Test Publishers:**
- ROS 1: Uses `rospy`, `rospy.Publisher()`, `rospy.Rate()`
- ROS 2: Uses `rclpy`, inherits from `Node` class, `self.create_publisher()`, `self.create_timer()`

**Commands:**
- ROS 1: `roslaunch`, `rostopic`, `rosnode`
- ROS 2: `ros2 launch`, `ros2 topic`, `ros2 node`

**Frontend Code:** Completely identical - rosbridge protocol ensures compatibility

## Development Workflow

### Modifying Frontend Code

When editing JavaScript files:
1. Make changes in the root directory version (ROS 1)
2. Copy the same changes to `ros2_version/` directory
3. Test with both ROS versions if possible

### Modifying Backend Code

- ROS 1 changes: Edit files in root directory
- ROS 2 changes: Edit files in `ros2_version/` directory
- Launch files and test publishers are version-specific

### Testing

Always test with the test publisher to verify functionality:
- Generates 4x4 meter wavy point cloud plane
- Simulates drone moving in circular trajectory
- Publishes at ~10 Hz

### Performance Considerations

- Point cloud limited to 500,000 points (auto-downsampled)
- Update throttling: 100ms minimum interval
- WebSocket throttle: 200ms
- Adjust `maxPoints` in pointcloud.js if performance issues occur
- Consider downsampling at the ROS publisher side for large point clouds

## Common Customizations

### Change ROI Default Size
Edit `js/roi_selector.js`:
```javascript
this.defaultSize = { x: 2, y: 2, z: 2 };  // Modify dimensions
```

### Change Published Topic Names
Edit `js/main.js` or modify in web UI sidebar (topics are configurable at runtime)

### Adjust Point Cloud Performance
Edit `js/pointcloud.js`:
```javascript
this.maxPoints = 500000;      // Maximum points
this.updateInterval = 100;    // Update frequency (ms)
```

## Troubleshooting

### rosbridge not connecting
- Verify rosbridge is running: `rosnode list | grep rosbridge` (ROS 1) or `ros2 node list | grep rosbridge` (ROS 2)
- Check port 9090 is not blocked: `netstat -tuln | grep 9090`
- Ensure ROS environment is sourced

### Point cloud not visible
- Check topic exists and is publishing: `rostopic hz /cloud_in` or `ros2 topic hz /cloud_in`
- Verify topic name matches in web UI
- Check browser console (F12) for errors
- Ensure point cloud has valid data

### Cannot pick points on point cloud
- Ensure point cloud is loaded and visible
- Click on dense areas of the point cloud
- Check raycaster threshold in main.js if needed

### Port conflicts between ROS 1 and ROS 2
- Both versions use port 9090 by default
- Cannot run simultaneously without changing port in launch files
- Modify port parameter in launch file if needed

## Important Notes

- This is NOT a git repository (no version control configured)
- Frontend files are duplicated between ROS 1 and ROS 2 versions - keep them synchronized
- The web UI must be served via HTTP server (not file://) for WebSocket to work
- Browser must support WebGL for Three.js rendering
- Touch screen devices are supported for mobile/tablet use
