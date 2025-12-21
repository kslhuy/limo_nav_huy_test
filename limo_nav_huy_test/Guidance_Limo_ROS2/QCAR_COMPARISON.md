# QCar vs Limo System Comparison

## Overview

You now have **TWO versions** of the system:

1. **Standard Limo System** - Uses `map` frame (original)
2. **QCar-Compatible System** - Uses `map_rotated` frame (matches QCar)

---

## System Architecture Comparison

### Standard Limo System (Original)

```
┌─────────────────────────────────────────┐
│ Terminal 1: limo_start.launch.py       │
│ └─ Hardware (motors, sensors)          │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│ Terminal 2: maptest.launch.py          │
│ ├─ Map Server                           │
│ ├─ AMCL (localization)                  │
│ ├─ waypoints node → /plan (map frame)  │
│ └─ RViz                                 │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│ Terminal 3: vehicle_main_ros            │
│ ├─ Subscribes: /plan (map frame)       │
│ ├─ GPS reads: map → base_link TF       │
│ └─ VehicleLogic + Stanley Controller   │
└─────────────────────────────────────────┘
```

### QCar-Compatible System (New)

```
┌─────────────────────────────────────────────┐
│ Terminal 1: limo_start.launch.py           │
│ └─ Hardware (motors, sensors)              │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│ Terminal 2: navigationV2V_qcar_frames.launch│
│ ├─ Map Server + AMCL                        │
│ ├─ TF: map → map_rotated (identity)        │
│ ├─ waypoints_qcar → /plan_qcar             │
│ │   (map_rotated frame)                     │
│ ├─ vehicle_main_ros_qcar                    │
│ │   Subscribes: /plan_qcar                  │
│ │   GPS reads: map_rotated → base_link TF   │
│ └─ RViz                                     │
└─────────────────────────────────────────────┘
```

---

## File Differences

| Component | Standard Limo | QCar-Compatible |
|-----------|---------------|-----------------|
| **Waypoint Node** | `waypoints.py` | `waypoints_qcar.py` |
| Path Topic | `/plan` | `/plan_qcar` |
| Frame | `map` | `map_rotated` |
| Transformation Order | Rotate → Scale → Translate | Scale → Translate → Rotate (QCar style) |
| Rotation Direction | Positive angle | Negative angle (QCar convention) |
| **Vehicle Controller** | `vehicle_main_ros.py` | `vehicle_main_ros_qcar.py` |
| GPS TF Lookup | `map → base_link` | `map_rotated → base_link` |
| Path Subscription | `/plan` | `/plan_qcar` |

---

## Coordinate Frame Details

### Why map_rotated?

QCar's cartographer creates maps with specific orientation that may not align with ROS conventions. The `map_rotated` frame allows:
- Decoupling map orientation from world coordinates
- Compatibility with QCar's SDCSRoadMap coordinate system
- Easier integration with QCar-based path planning

### TF Tree Structure

**Standard Limo:**
```
map → odom → base_link
```

**QCar-Compatible:**
```
map → map_rotated → odom → base_link
         ↑
    (identity transform)
```

---

## Transformation Pipeline Comparison

### Standard Limo (waypoints.py)

```python
# 1. Scale
x = raw_x * scale
y = raw_y * scale

# 2. Mirror/Flip (optional)
if mirror_x: x = -x
if mirror_y: y = -y

# 3. Rotate
x_rot = x*cos(θ) - y*sin(θ)
y_rot = x*sin(θ) + y*cos(θ)

# 4. Translate
x_final = x_rot + dx
y_final = y_rot + dy
```

### QCar-Compatible (waypoints_qcar.py)

```python
# 1. Scale
wp = [raw_x * scale, raw_y * scale]

# 2. Translate (BEFORE rotation, like QCar)
wp = wp + [tx, ty]

# 3. Rotate (NEGATIVE angle, QCar convention)
angle = -rotation_offset * π/180
wp_final = wp @ R(-θ)
```

---

## How to Run

### 1. Build First (Required for both systems)

```bash
cd ~/agilex_ws
colcon build --packages-select limo_nav_huy_test --symlink-install
source install/setup.bash
```

### 2A. Run Standard Limo System

**Terminal 1:**
```bash
ros2 launch limo_bringup limo_start.launch.py
```

**Terminal 2:**
```bash
ros2 launch limo_nav_huy_test maptest.launch.py
```

**Terminal 3:**
```bash
ros2 run limo_nav_huy_test vehicle_main_ros
```

**Adjust Parameters:**
```bash
ros2 param set /waypoints scale 1.0
ros2 param set /waypoints theta_deg 80.0
ros2 param set /waypoints dx 0.7
ros2 param set /waypoints dy 2.0
ros2 param set /waypoints mirror_x True
```

### 2B. Run QCar-Compatible System (RECOMMENDED)

**Terminal 1:**
```bash
ros2 launch limo_bringup limo_start.launch.py
```

**Terminal 2:**
```bash
ros2 launch limo_nav_huy_test navigationV2V_qcar_frames.launch.py
```

**Adjust Parameters:**
```bash
ros2 param set /waypoints_qcar scale 1.0
ros2 param set /waypoints_qcar rotation_offset 90.0
ros2 param set /waypoints_qcar translation_offset [0.0, 0.0]
```

---

## Topic Differences

### Standard Limo Topics

| Topic | Type | Publisher | Subscriber | Frame |
|-------|------|-----------|------------|-------|
| `/plan` | Path | waypoints | vehicle_main_ros | map |
| `/waypoints_viz` | MarkerArray | waypoints | RViz | map |

### QCar-Compatible Topics

| Topic | Type | Publisher | Subscriber | Frame |
|-------|------|-----------|------------|-------|
| `/plan_qcar` | Path | waypoints_qcar | vehicle_main_ros_qcar | map_rotated |
| `/waypoints_viz_qcar` | MarkerArray | waypoints_qcar | RViz | map_rotated |

---

## Verification Commands

### Check TF Frames

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map map_rotated
ros2 run tf2_ros tf2_echo map_rotated base_link
```

### Monitor Topics

```bash
# Standard system
ros2 topic echo /plan
ros2 topic hz /plan

# QCar system
ros2 topic echo /plan_qcar
ros2 topic hz /plan_qcar
```

### Check Node Status

```bash
# List running nodes
ros2 node list

# Check specific node
ros2 node info /waypoints_qcar
ros2 node info /vehicle_main_ros_qcar
```

---

## Which System Should You Use?

### Use **Standard Limo System** if:
- You want simple, direct ROS coordinate system
- You're familiar with ROS map frame conventions
- You don't need QCar compatibility

### Use **QCar-Compatible System** if:
- You want exact QCar coordinate system behavior
- You're migrating from QCar platform
- You need SDCSRoadMap compatibility
- You want to match QCar transformation pipeline

---

## Troubleshooting

### Problem: Path not visible in RViz

**Solution:**
- For Standard: Add `/plan` and `/waypoints_viz` topics (frame: `map`)
- For QCar: Add `/plan_qcar` and `/waypoints_viz_qcar` topics (frame: `map_rotated`)

### Problem: Vehicle not following path

**Check:**
1. GPS adapter receiving pose: `ros2 topic echo /amcl_pose`
2. Path being published: `ros2 topic echo /plan_qcar`
3. TF transform exists: `ros2 run tf2_ros tf2_echo map_rotated base_link`
4. Controller receiving path: Check vehicle_main_ros_qcar logs

### Problem: Waypoints in wrong location

**Adjust parameters:**
```bash
# QCar system
ros2 param set /waypoints_qcar rotation_offset 90.0  # Try different angles
ros2 param set /waypoints_qcar translation_offset [0.5, 0.5]  # Shift path
ros2 param set /waypoints_qcar scale 0.95  # Scale down/up
```

---

## Summary

✅ **Your QCar-compatible implementation is correct!**

**Key Features:**
- Proper `map_rotated` frame usage
- QCar transformation pipeline (Scale → Translate → Rotate)
- Negative angle rotation (QCar convention)
- Separate topics (`/plan_qcar` vs `/plan`)
- Compatible with QCar's SDCSRoadMap

**Both systems work for path following without vision!**
