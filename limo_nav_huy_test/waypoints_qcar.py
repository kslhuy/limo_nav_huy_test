"""
QCar-Style Waypoint Publisher
Uses map_rotated frame and QCar's transformation order: Scale → Translate → Rotate (negative angle)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from hal.products.mats import SDCSRoadMap
import math
import numpy as np

class WaypointsQCar(Node):
    def __init__(self):
        super().__init__('waypoints_qcar')
        
        # --- QCar-style PARAMETERS ---
        # Translation offset (applied BEFORE rotation, like QCar)
        self.declare_parameter('translation_offset', [0.0, 0.0])
        
        # Rotation offset in degrees (applied with negative sign, like QCar)
        self.declare_parameter('rotation_offset', 90.0)
        
        # Scale factor
        self.declare_parameter('scale', 1.0)
        
        # Node sequence from SDCSRoadMap
        self.declare_parameters(
            namespace='',
            parameters=[('nodeSequence', [10, 2, 4, 6, 8, 10])]
        )
        
        self.nodeSequence = self.get_parameter("nodeSequence").value
        self.roadmap = SDCSRoadMap(leftHandTraffic=False, useSmallMap=True)
        self.waypointSequence = self.roadmap.generate_path(self.nodeSequence) * 0.975
        self.waypoints_x = self.waypointSequence[0]
        self.waypoints_y = self.waypointSequence[1]

        # Publishers for visualization and planning
        self.vis_path_pub = self.create_publisher(MarkerArray, "/waypoints_viz_qcar", 10)
        self.path_pub = self.create_publisher(Path, "/plan_qcar", 10)
        
        self.timer = self.create_timer(0.5, self.pub)
        
        self.get_logger().info("QCar-style waypoint publisher initialized")
        self.get_logger().info(f"Node sequence: {self.nodeSequence}")

    def pub(self):
        # Get parameters (QCar style)
        translation_offset = self.get_parameter('translation_offset').value
        rotation_offset = self.get_parameter('rotation_offset').value
        scale = self.get_parameter('scale').value
        
        # Prepare messages
        marker_array = MarkerArray()
        path_msg = Path()
        path_msg.header.frame_id = "map_rotated"  # QCar uses map_rotated frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (raw_x, raw_y) in enumerate(zip(self.waypoints_x, self.waypoints_y)):
            
            # === QCar Transformation Pipeline ===
            # 1. Scale waypoints
            wp_scaled = np.array([raw_x * scale, raw_y * scale])
            
            # 2. Translate (QCar applies translation BEFORE rotation)
            t = np.array([translation_offset[0], translation_offset[1]])
            wp_translated = wp_scaled + t
            
            # 3. Rotate (QCar uses NEGATIVE angle)
            angle_rad = -rotation_offset * np.pi / 180.0
            R_QLabs_ROS = np.array([
                [np.cos(angle_rad), -np.sin(angle_rad)],
                [np.sin(angle_rad),  np.cos(angle_rad)]
            ])
            wp_final = wp_translated @ R_QLabs_ROS
            
            x_final = wp_final[0]
            y_final = wp_final[1]
            
            # --- Visual Markers ---
            marker = Marker()
            marker.header.frame_id = "map_rotated"
            marker.type = Marker.SPHERE
            marker.id = i
            marker.action = Marker.ADD 
            marker.pose.position.x = x_final
            marker.pose.position.y = y_final
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.r = 1.0  # Red markers for QCar style
            marker.color.g = 0.5
            marker.lifetime = Duration(sec=1, nanosec=0)
            marker_array.markers.append(marker)

            # --- Robot Path ---
            pose = PoseStamped()
            pose.header.frame_id = "map_rotated"
            pose.pose.position.x = x_final
            pose.pose.position.y = y_final
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.vis_path_pub.publish(marker_array)
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointsQCar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
