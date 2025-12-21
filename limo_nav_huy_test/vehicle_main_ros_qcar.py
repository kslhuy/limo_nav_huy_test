"""
ROS 2 Version of vehicle_main.py (Full Vehicle Control System) - QCar Coordinate Style

This version uses QCar's coordinate system approach:
- map_rotated frame instead of map
- QCar-style transformation pipeline
- Compatible with QCar's SDCSRoadMap coordinate system
"""

import sys
import os
import time
import numpy as np
from threading import Event
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, PoseStamped , PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from limo_msgs.msg import LimoStatus
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

# ===== ADD PATH TO QCAR FOLDER =====
current_dir = os.path.dirname(os.path.abspath(__file__))
qcar_path = os.path.join(current_dir, "multi_vehicle_RealCar")

if os.path.exists(qcar_path):
    if qcar_path not in sys.path:
        sys.path.insert(0, qcar_path)
        print(f"[PATH] Added to sys.path: {qcar_path}")
else:
    print(f"[PATH ERROR] QCar path not found: {qcar_path}")
    raise FileNotFoundError(f"Required directory not found: {qcar_path}")

from qcar.config_main import VehicleMainConfig
from qcar.vehicle_logic import VehicleLogic
from qcar.command_types import CommandType
from limo.limo import ROSQCarAdapter, ROSGPSAdapterQCar


# ===== MAIN ROS NODE (QCar Style) =====
class VehicleControlFullSystemQCar(Node):
    """ROS 2 wrapper for complete VehicleLogic system - QCar coordinate style"""
    
    def __init__(self):
        super().__init__('vehicle_control_full_system_qcar')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("="*70)
        self.get_logger().info("Initializing Full Vehicle Control System (QCar Coordinate Style)")
        self.get_logger().info("="*70)
        
        # ===== ROS PARAMETERS =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('car_id', 3),
                ('v_ref', 0.75),
                ('controller_rate', 100),
                ('calibrate', False),
                ('path_number', 0),
                ('no_steering', False),
                ('config_file', ''),
                ('log_dir', ''),
                ('data_log_dir', ''),
                ('initial_x', 0.0),
                ('initial_y', 0.0),
                ('initial_yaw', 0.0),
            ]
        )
        
        car_id = self.get_parameter('car_id').value
        v_ref = self.get_parameter('v_ref').value
        controller_rate = self.get_parameter('controller_rate').value
        calibrate = self.get_parameter('calibrate').value
        path_number = self.get_parameter('path_number').value
        no_steering = self.get_parameter('no_steering').value
        config_file = self.get_parameter('config_file').value
        log_dir = self.get_parameter('log_dir').value
        data_log_dir = self.get_parameter('data_log_dir').value
        initial_x = self.get_parameter('initial_x').value
        initial_y = self.get_parameter('initial_y').value
        initial_yaw = self.get_parameter('initial_yaw').value
        
        self.get_logger().info(f"QCar Style - Car ID: {car_id}, v_ref: {v_ref}, rate: {controller_rate} Hz")
        
        # ===== INITIALIZE DATA STORAGE =====
        self.latest_odom = None
        self.latest_imu = None
        self.latest_limo_status = None
        
        # ===== ROS PUBLISHERS =====
        self.motor_pub = self.create_publisher(Twist, "/cmd_vel", 30)
        
        # ===== CREATE ROS ADAPTERS (QCar Style) =====
        self.qcar_adapter = ROSQCarAdapter(self)
        self.gps_adapter = ROSGPSAdapterQCar(self, self.tf_buffer)
        self.get_logger().info("✓ ROS hardware adapters created (QCar style)")
        
        # Send initial pose to AMCL
        initial_x = -1.28205
        initial_y = -0.45991
        initial_yaw = -0.7330 * (180.0 / 3.1415926)
        self.gps_adapter.send_initial_pose(initial_x, initial_y, initial_yaw)
        
        # ===== ROS SUBSCRIPTIONS =====
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_callback, 10
        )
        self.limo_status_sub = self.create_subscription(
            LimoStatus, '/limo_status', self._limo_status_callback, 10
        )
        # Subscribe to QCar-style path topic
        self.path_sub = self.create_subscription(
            Path, '/plan_qcar', self._path_callback, 10
        )
        
        # ===== LOAD CONFIGURATION =====
        self.get_logger().info("Loading VehicleMainConfig...")
        
        if config_file and config_file.strip():
            config_path = config_file
        else:
            config_path = os.path.join(qcar_path, 'qcar', 'config_vehicle_main.yaml')
        
        if os.path.exists(config_path):
            if config_path.endswith('.json'):
                config = VehicleMainConfig.from_json(config_path)
            elif config_path.endswith('.yaml') or config_path.endswith('.yml'):
                config = VehicleMainConfig.from_yaml(config_path)
            else:
                raise ValueError(f"Invalid config file format: {config_path}")
            self.get_logger().info(f"Loaded config from: {config_path}")
        else:
            config = VehicleMainConfig()
            
        config.network.car_id = car_id
        config.timing.controller_update_rate = controller_rate
        config.path.path_number = path_number
        
        if log_dir and log_dir.strip():
            config.logging.log_dir = log_dir
        else:
            config.logging.log_dir = os.path.join(qcar_path, 'qcar', 'logs')
        
        if data_log_dir and data_log_dir.strip():
            config.logging.data_log_dir = data_log_dir
        else:
            config.logging.data_log_dir = os.path.join(qcar_path, 'qcar', 'data_logs')
        
        self.calibrate = calibrate
        
        # ===== CREATE VEHICLE LOGIC =====
        self.kill_event = Event()
        self.get_logger().info("Creating VehicleLogic instance...")
        
        self.vehicle_logic = VehicleLogic(config, self.kill_event)
        
        # Replace hardware interfaces with ROS adapters
        self.vehicle_logic.qcar = self.qcar_adapter
        self.vehicle_logic.gps = self.gps_adapter
        self.vehicle_logic.v_ref = v_ref
        
        self._setup_ros_initialization_override()
        self.get_logger().info("VehicleLogic created successfully")
        
        # ===== ROS TIMERS FOR CONTROL LOOPS =====
        self.observer_rate = 150
        self.observer_timer = self.create_timer(1.0/self.observer_rate, self._observer_callback)
        
        control_period = 1.0 / controller_rate
        self.control_timer = self.create_timer(control_period, self._control_callback)
        
        self.last_control_time = time.time()
        self.last_observer_time = time.time()
        self.control_dt = control_period
        
        # ROS topic readiness flags
        self.topics_ready = False
        self.pose_received = False
        self.joint_received = False
        self.path_received = False
        
        self.init_check_timer = self.create_timer(0.1, self._check_initialization)
        self.tf_update_timer = self.create_timer(0.1, self._update_gps_from_tf)
        
        self.get_logger().info("="*70)
        self.get_logger().info("Full Vehicle Control System Ready! (QCar Coordinate Style)")
        self.get_logger().info("Using map_rotated frame")
        self.get_logger().info("="*70)
        
    # ===== ROS CALLBACKS =====
    
    def _odom_callback(self, msg: Odometry):
        """Update motor tach from odometry data"""
        linear_vel = msg.twist.twist.linear.x
        self.qcar_adapter.update_motor_tach(linear_vel)
        
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info("✓ Odometry topic connected")
    
    def _limo_status_callback(self, msg: LimoStatus):
        """Handle Limo status updates"""
        self.qcar_adapter.update_Limo_status(msg)
        
        if not self.joint_received:
            self.joint_received = True
            self.get_logger().info("✓ Limo status topic connected")
            
    def _imu_callback(self, msg: Imu):
        """Update gyroscope data"""
        gyro_z = msg.angular_velocity.z
        accel_x = msg.linear_acceleration.x
        self.qcar_adapter.update_gyro(gyro_z)
        self.qcar_adapter.update_accel(accel_x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        
    def _path_callback(self, msg: Path):
        """Receive path from waypoints_qcar node (already in map_rotated frame)"""
        if len(msg.poses) < 2:
            self.get_logger().warning("Received path with less than 2 waypoints, ignoring")
            return
        
        waypoints_x = []
        waypoints_y = []
        
        for pose_stamped in msg.poses:
            waypoints_x.append(pose_stamped.pose.position.x)
            waypoints_y.append(pose_stamped.pose.position.y)
        
        waypoint_sequence = np.array([waypoints_x, waypoints_y])
        self.vehicle_logic.waypoint_sequence = waypoint_sequence
        
        if not self.path_received:
            self.path_received = True
            self.get_logger().info(f"✓ QCar-style path received: {waypoint_sequence.shape[1]} waypoints")
            self.get_logger().info(f"  Start: ({waypoints_x[0]:.2f}, {waypoints_y[0]:.2f})")
            self.get_logger().info(f"  End: ({waypoints_x[-1]:.2f}, {waypoints_y[-1]:.2f})")
    
    # ===== INITIALIZATION CHECK =====
    
    def _check_initialization(self):
        """Check if ROS topics are connected"""
        if self.topics_ready:
            return
        
        if self.pose_received and self.joint_received and self.path_received:
            if not self.topics_ready:
                self.get_logger().info("="*70)
                self.get_logger().info("✓ ALL ROS TOPICS READY - State Machine Can Initialize")
                self.get_logger().info("="*70)
                self.topics_ready = True
                self.init_check_timer.cancel()
                
                if hasattr(self.vehicle_logic, '_ros_topics_ready'):
                    self.vehicle_logic._ros_topics_ready = True
    
    def _update_gps_from_tf(self):
        """Update GPS adapter from TF transform (map_rotated → base_link)"""
        self.gps_adapter.update_from_tf()
    
    # ===== OBSERVER UPDATE LOOP =====
    
    def _observer_callback(self):
        """High-rate observer update"""
        if not self.topics_ready:
            return
            
        current_time = time.time()
        dt = current_time - self.last_observer_time
        self.last_observer_time = current_time
        
        try:
            self.vehicle_logic._update_sensor_data(dt)
            self.vehicle_logic._observer_update(dt)
        except Exception as e:
            self.get_logger().error(f"Observer update error: {e}")
    
    # ===== CONTROL LOOP =====
    
    def _control_callback(self):
        """Main control loop"""
        if not self.topics_ready:
            return
            
        current_time = time.time()
        dt = current_time - self.last_control_time
        self.last_control_time = current_time
        
        try:
            success = self.vehicle_logic._control_logic_update(dt)
            
            if not success:
                self.get_logger().warning("Control logic update failed")

            self.vehicle_logic._send_telemetry_to_ground_station()
            self.vehicle_logic._process_queued_commands()
            self.vehicle_logic._broadcast_v2v_state()
            self.vehicle_logic.loop_counter += 1

        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
            import traceback
            traceback.print_exc()
        
    def _setup_ros_initialization_override(self):
        """Setup ROS-specific initialization"""
        self.vehicle_logic._ros_mode = True
        self.vehicle_logic._ros_topics_ready = False
        self.get_logger().info("✓ ROS-specific initialization mode enabled")
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down VehicleControlFullSystemQCar...")
        self.kill_event.set()
        
        if hasattr(self, 'vehicle_logic'):
            self.vehicle_logic._shutdown()
            
        super().destroy_node()


# ===== MAIN ENTRY POINT =====
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VehicleControlFullSystemQCar()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutdown requested (Ctrl+C)")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
