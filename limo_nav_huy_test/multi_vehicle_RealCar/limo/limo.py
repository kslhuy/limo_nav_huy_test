"""
Mock QCar module for ROS integration
This replaces the actual Quanser QCar hardware with ROS adapters
"""

import time
import numpy as np
import math
import rclpy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

# Flag to indicate we're NOT using physical QCar (using ROS instead)
IS_LIMO_CAR = True

# ===== ROS ADAPTER FOR QCAR HARDWARE =====
class ROSQCarAdapter:
    """Adapter to replace QCar hardware interface with ROS topics"""
    
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self._motor_tach = 0.0
        self._gyro_z = 0.0
        self._accel_x = 0.0
        self._accel_y = 0.0 
        self._accel_z = 0.0
        self._last_update = time.time()
        self.motorCurrent = 0.0
        self.batteryVoltage = 0.0
        self._gyroscope = np.zeros(3)
        self._accelerometer = np.zeros(3)
        self.motorEncoder = []
        self._motor_tach_conv = 0.0
        self.CPS_TO_MPS = getattr(ros_node, 'CPS_TO_MPS', 1.0)
        self.motor_pub = ros_node.motor_pub
    def read(self):
        """Mimic QCar.read() interface - returns sensor array"""
        # QCar.read() returns: [motor_tach, encoder, gyro_x, gyro_y, gyro_z, ...]
        # We'll return a simplified version with what we have from ROS
        # return [
        #     self._motor_tach,  # index 0: motor tachometer
        #     0.0,               # index 1: encoder (not used)
        #     0.0,               # index 2: gyro_x
        #     0.0,               # index 3: gyro_y
        #     self._gyro_z,      # index 4: gyro_z
        # ]
    def read_write_std(self, throttle=0.0, steering=0.0, LEDs=None):
        """Mimic QCar.read_write_std() interface"""
        self.write(throttle, steering)
        
    def write(self, throttle=0.0, steering=0.0):
        """Publish to ROS using Twist message for Limo"""
        msg = Twist()
        msg.linear.x = float(throttle)
        msg.angular.z = float(steering)
        self.motor_pub.publish(msg)

    def update_Limo_status(self, status_msg):
        """Update internal state from LimoStatus message"""
        self.batteryVoltage = status_msg.battery_voltage
        
    def update_motor_tach(self, value):
        """Update motor tachometer from ROS joint callback"""
        self._motor_tach = value
        self._last_update = time.time()
        
    def update_gyro(self, gyro_z):
        """Update gyroscope from ROS IMU callback"""
        self._gyro_z = gyro_z

    def update_accel(self, accel_x, accel_y, accel_z):
        """Update accelerometer from ROS IMU callback"""
        self._accel_x = accel_x
        self._accel_y = accel_y
        self._accel_z = accel_z

    @property
    def motorTach(self):
        return self._motor_tach_conv

    @property
    def gyroscope(self):
        return self._gyroscope

    @property
    def accelerometer(self):
        return self._accelerometer


# ===== ROS GPS ADAPTER (QCar Style) =====
class ROSGPSAdapterQCar:
    """
    Adapter to replace QCarGPS with ROS pose data
    QCar Style: Uses map_rotated frame
    """
    def __init__(self, ros_node, tf_buffer, initialPose=None, **kwargs):
        self.ros_node = ros_node
        self.tf_buffer = tf_buffer
        
        if initialPose is None:
            initialPose = [0.0, 0.0, 0.0]

        self.position = np.array([initialPose[0], initialPose[1], 0.0])
        self.orientation = np.array([0.0, 0.0, initialPose[2]])
        self._last_update = time.time()
        self._last_read_time = 0.0  # Track when data was last read
        
        self.initial_pose_pub = ros_node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
    #  Like calibrateGPS in QCarGPS
    def send_initial_pose(self, x, y, yaw_deg):
        """Send initial pose to AMCL (still uses 'map' frame for AMCL)"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # AMCL expects map frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
        msg.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
        
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05

        self.initial_pose_pub.publish(msg)
        self.ros_node.get_logger().info(f"Sent initial pose: x={x}, y={y}, yaw={yaw_deg}°")
        
    def update_from_tf(self):
        """Update pose from TF lookup - QCar style uses map_rotated frame"""
        try:
            # QCar method: lookup map_rotated → base_link
            t = self.tf_buffer.lookup_transform(
                'map_rotated', 'base_link', rclpy.time.Time())
            
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            
            rotation = [t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w]
            roll, pitch, yaw = R.from_quat(rotation).as_euler('xyz')
            
            self.update_pose(x, y, yaw)
            
        except Exception as e:
            pass
        
    def readGPS(self):
        """Mimic QCarGPS.readGPS() interface - returns True only if new data available"""
        if self._last_update > self._last_read_time:
            self._last_read_time = time.time()
            return True
        return False
    
    def update_pose(self, x, y, yaw):
        """Update pose from ROS EKF callback"""
        self.position = np.array([x, y, 0.0])
        self.orientation = np.array([0.0, 0.0, yaw])
        self._last_update = time.time()



# Export the classes
__all__ = ['IS_LIMO_CAR', 'ROSQCarAdapter', 'ROSGPSAdapterQCar']
