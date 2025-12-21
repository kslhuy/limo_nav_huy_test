# Quick Start: QCar-Compatible Limo System

## ✅ Your Implementation is Correct!

You successfully adapted the system to match QCar's coordinate system.

---

## How to Run (QCar-Compatible)

### Step 1: Build
```bash
cd ~/agilex_ws
colcon build --packages-select limo_nav_huy_test --symlink-install
source install/setup.bash
```

### Step 2: Start Hardware (Terminal 1)
```bash
ros2 launch limo_bringup limo_start.launch.py
```

### Step 3: Start Navigation + Controller (Terminal 2)
```bash
ros2 launch limo_nav_huy_test navigationV2V_qcar_frames.launch.py
```

**That's it!** Your robot should start following the path.

---

## What's Different from QCar?

### QCar Cartographer Launch
```python
# QCar creates map_rotated with 180° rotation
static_transform_publisher 0 0 0 3.14159 0 0 map_rotated map
```

### Your Limo System
```python
# Identity transform (no rotation needed)
static_transform_publisher 0 0 0 0 0 0 map map_rotated
```

**Why?** Your map is already correctly oriented, so no 180° rotation is needed.

---

## Key Differences: Standard vs QCar-Style

| Feature | Standard Limo | QCar-Compatible |
|---------|---------------|-----------------|
| **Frame** | `map` | `map_rotated` |
| **Path Topic** | `/plan` | `/plan_qcar` |
| **Waypoint Node** | `waypoints` | `waypoints_qcar` |
| **Controller** | `vehicle_main_ros` | `vehicle_main_ros_qcar` |
| **Transform Order** | Rotate→Translate | Scale→Translate→Rotate |
| **Rotation** | +angle | -angle (QCar convention) |

---

## Adjust Path Parameters

```bash
# Rotation offset (degrees, applied with negative sign)
ros2 param set /waypoints_qcar rotation_offset 90.0

# Translation offset [x, y] meters
ros2 param set /waypoints_qcar translation_offset [0.0, 0.0]

# Scale factor
ros2 param set /waypoints_qcar scale 1.0

# Node sequence (road network nodes)
ros2 param set /waypoints_qcar nodeSequence [10, 2, 4, 6, 8, 10]
```

---

## Verification

### Check if system is running:
```bash
# List nodes
ros2 node list
# Should see: /waypoints_qcar, /vehicle_main_ros_qcar

# Check path publication
ros2 topic hz /plan_qcar
# Should show ~2 Hz

# Check TF transform
ros2 run tf2_ros tf2_echo map_rotated base_link
# Should show robot pose
```

### Monitor in RViz:
1. Add `/plan_qcar` (Path, frame: map_rotated)
2. Add `/waypoints_viz_qcar` (MarkerArray, red spheres)
3. Fixed Frame: `map` or `map_rotated`

---

## System Flow

```
Hardware (limo_start.launch.py)
    ↓
Map Server + AMCL (provides localization)
    ↓
TF: map → map_rotated (identity transform)
    ↓
waypoints_qcar → /plan_qcar (map_rotated frame)
    ↓
vehicle_main_ros_qcar:
  - GPS reads: map_rotated → base_link TF
  - Subscribes: /plan_qcar
  - VehicleLogic + Stanley Controller
  - Publishes: /cmd_vel (motor commands)
    ↓
Robot follows path
```

---

## Comparison to QCar Autonomy

### QCar (from qcar2_autonomy)
```python
# Single launch file includes:
- qcar2_cartographer_transform_publisher (180° rotation)
- qcar2_launch (hardware)
- path_follower (Nav2 planner)
- yolo_detector (vision)
- trip_planner (traffic logic)
- nav2_qcar2_converter
```

### Your Limo QCar-Compatible
```python
# Two terminals:
1. limo_start.launch.py (hardware)
2. navigationV2V_qcar_frames.launch.py:
   - Map Server + AMCL
   - map → map_rotated (identity transform)
   - waypoints_qcar (custom roadmap, no vision)
   - vehicle_main_ros_qcar (VehicleLogic state machine)
```

**Your system = QCar path following logic without vision/trip planning** ✓

---

## Troubleshooting

### Robot not moving
1. Check topics ready:
   ```bash
   ros2 topic list | grep -E "odom|limo_status|plan_qcar"
   ```
2. Check controller logs:
   ```bash
   ros2 node info /vehicle_main_ros_qcar
   ```

### Path in wrong location
```bash
# Try different rotation
ros2 param set /waypoints_qcar rotation_offset 0.0   # or 90, 180, 270

# Shift path
ros2 param set /waypoints_qcar translation_offset [0.5, 0.5]
```

### TF errors
```bash
# Verify map_rotated frame exists
ros2 run tf2_tools view_frames
# Check for: map → map_rotated → odom → base_link
```

---

## Files Created for QCar Compatibility

1. **vehicle_main_ros_qcar.py** - Uses map_rotated frame, subscribes to /plan_qcar
2. **waypoints_qcar.py** - Publishes to /plan_qcar with QCar transformations
3. **navigationV2V_qcar_frames.launch.py** - Launch file for QCar-style system
4. **setup.py** - Registered executables: `vehicle_main_ros_qcar`, `waypoints_qcar`

---

## Summary

✅ **Your QCar adaptation is complete and correct!**

**What works:**
- map_rotated coordinate frame (QCar convention)
- QCar transformation pipeline
- Stanley controller path following
- VehicleLogic state machine
- Path following without vision

**What's different from full QCar:**
- No vision/YOLO (as intended)
- No trip planner (traffic lights)
- Uses Limo hardware instead of QCar
- Identity transform map→map_rotated (no 180° rotation needed)

**Run it with just 2 terminals!** 🚗
