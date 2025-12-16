# Vehicle Main ROS - Limo Integration

## Overview
This package integrates the multi-vehicle control system with the Limo robot platform using ROS 2.

## Key Changes Made

### 1. **Path Configuration**
- **Fixed**: Path to `multi_vehicle_RealCar` is now correctly set to the package directory
- **Location**: `/home/agilex/agilex_ws/src/limo_nav_huy_test/limo_nav_huy_test/multi_vehicle_RealCar`

### 2. **Message Types - Adapted for Limo**
- **Removed**: `qcar2_interfaces.msg.MotorCommands` (doesn't exist)
- **Using**: Standard ROS 2 messages:
  - `geometry_msgs.msg.Twist` for motor commands
  - `nav_msgs.msg.Odometry` for odometry
  - `sensor_msgs.msg.Imu` for IMU data
  - `limo_msgs.msg.LimoStatus` for Limo-specific status

### 3. **Topic Mappings**

#### Publishers:
- **Motor Commands**: `/cmd_vel` (Twist)
  - `msg.linear.x` = throttle (linear velocity in m/s)
  - `msg.angular.z` = steering (angular velocity in rad/s)

#### Subscribers:
- **Odometry**: `/odom` (Odometry)
  - Provides position (x, y, yaw) and velocity
- **IMU**: `/imu` (Imu)
  - Provides gyroscope data (angular velocity)
- **Limo Status**: `/limo_status` (LimoStatus)
  - Vehicle state, battery, error codes, motion mode
- **Commands**: `/vehicle_{car_id}/commands` (String)
  - For Ground Station or remote commands

### 4. **Hardware Adapters**

#### ROSQCarAdapter:
- **`read()`**: Returns sensor array from ROS subscriptions
  - Motor tachometer from odometry
  - Gyroscope Z from IMU
- **`write(throttle, steering)`**: Publishes Twist to `/cmd_vel`

#### ROSGPSAdapter:
- **`read()`**: Returns [x, y, z] position
- **`update_pose(x, y, yaw)`**: Updates from odometry callback

### 5. **Dependencies**
Updated `package.xml`:
```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>limo_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
```

### 6. **Entry Point**
Added to `setup.py`:
```python
'vehicle_main_ros = limo_nav_huy_test.vehicle_main_ros:main'
```

## Usage

### Build the Package:
```bash
cd ~/agilex_ws
colcon build --packages-select limo_nav_huy_test --symlink-install
source install/setup.bash
```

### Run the Node:
```bash
ros2 run limo_nav_huy_test vehicle_main_ros
```

### With Parameters:
```bash
ros2 run limo_nav_huy_test vehicle_main_ros \
  --ros-args \
  -p car_id:=0 \
  -p v_ref:=0.5 \
  -p controller_rate:=100
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `car_id` | int | 0 | Vehicle ID for multi-vehicle scenarios |
| `v_ref` | float | 0.75 | Reference velocity (m/s) |
| `controller_rate` | int | 100 | Control loop frequency (Hz) |
| `calibrate` | bool | false | Recalibrate vehicle before starting |
| `path_number` | int | 0 | Node configuration (0, 1, 2) |
| `no_steering` | bool | false | Disable steering control |
| `config_file` | string | '' | Custom config file path |
| `log_dir` | string | '' | Custom log directory |
| `data_log_dir` | string | '' | Custom data log directory |

## Architecture

```
VehicleControlFullSystem (ROS Node)
├── ROSQCarAdapter (Hardware abstraction)
│   ├── Publishes to /cmd_vel (Twist)
│   └── Reads from /odom and /imu
├── ROSGPSAdapter (Position abstraction)
│   └── Reads from /odom
├── VehicleLogic (Original control system)
│   ├── V2V Manager (UDP multicast)
│   ├── Ground Station Client (TCP)
│   ├── Command Handler
│   ├── State Machine
│   ├── Observer (200 Hz)
│   └── Controller (100 Hz)
└── ROS Timers
    ├── Observer callback (200 Hz)
    ├── Control callback (100 Hz)
    └── Initialization check (10 Hz)
```

## Communication Preserved

### Original TCP/UDP (Not ROS):
- **V2V Communication**: UDP multicast for vehicle-to-vehicle
- **Ground Station**: TCP client for telemetry and commands

### ROS Topics:
- Motor control
- Sensor data (odometry, IMU)
- Vehicle status

## Initialization Sequence

1. **ROS Node Startup**: Declares parameters, creates publishers/subscribers
2. **Adapter Creation**: ROSQCarAdapter and ROSGPSAdapter replace hardware
3. **VehicleLogic Creation**: Original control system initialized
4. **Topic Waiting**: Waits for `/odom` and `/limo_status` to be ready
5. **Control Start**: Observer (200 Hz) and control (100 Hz) loops begin

## Troubleshooting

### No Odometry Data:
```bash
# Check if limo_base is running
ros2 topic list | grep odom
ros2 topic echo /odom --once
```

### Limo Not Responding:
```bash
# Check if limo_base is connected
ros2 topic list | grep limo_status
ros2 topic echo /limo_status --once
```

### Path Errors:
- Ensure `multi_vehicle_RealCar` exists at:
  `/home/agilex/agilex_ws/src/limo_nav_huy_test/limo_nav_huy_test/multi_vehicle_RealCar`
- Contains `qcar/` folder with Python modules

## Next Steps

To integrate with the full system:
1. Ensure `limo_base` is running
2. Start the vehicle control node
3. Connect Ground Station (if needed)
4. Send commands via Ground Station or ROS topics

## Contact
For issues related to:
- **Limo Hardware**: Check `limo_ros2` package
- **Vehicle Logic**: Check `multi_vehicle_RealCar` directory
- **ROS Integration**: This package (`limo_nav_huy_test`)
