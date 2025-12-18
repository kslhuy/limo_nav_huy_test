import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from hal.products.mats import SDCSRoadMap
import math

class Waypoints(Node):
    def __init__(self):
        super().__init__('waypoints')
        
        # --- DYNAMIC PARAMETERS ---
        self.declare_parameter('dx', 0.7)        
        self.declare_parameter('dy', 2.0)        
        self.declare_parameter('theta_deg', 80.0) 
        self.declare_parameter('scale', 1.0)
        
        # NEW: FLIP PARAMETERS
        self.declare_parameter('mirror_x', False) # Flip horizontally?
        self.declare_parameter('mirror_y', False) # Flip vertically?
        self.declare_parameter('swap_xy', False)  # Swap X and Y axes?
        
        self.declare_parameters(
            namespace='',
            parameters=[('nodeSequence', [10, 2,4, 6, 8, 10])]
        )
        
        self.nodeSequence = self.get_parameter("nodeSequence").value
        self.roadmap = SDCSRoadMap(leftHandTraffic=False , useSmallMap=True)
        self.waypointSequence = self.roadmap.generate_path(self.nodeSequence)
        self.waypoints_x = self.waypointSequence[0]
        self.waypoints_y = self.waypointSequence[1]

        self.vis_path_pub = self.create_publisher(MarkerArray, "/waypoints_viz", 10)
        self.path_pub = self.create_publisher(Path, "/plan", 10)
        
        self.timer = self.create_timer(0.5, self.pub)

    def pub(self):
        # 1. GET PARAMETERS
        dx = self.get_parameter('dx').value
        dy = self.get_parameter('dy').value
        theta_deg = self.get_parameter('theta_deg').value
        scale = self.get_parameter('scale').value
        
        # NEW: Get Flip Params
        mirror_x = self.get_parameter('mirror_x').value
        mirror_y = self.get_parameter('mirror_y').value
        swap_xy = self.get_parameter('swap_xy').value
        
        angle_radians = math.radians(theta_deg)

        marker_array = MarkerArray()
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (raw_x, raw_y) in enumerate(zip(self.waypoints_x, self.waypoints_y)):
            
            # A. APPLY SCALE
            x_calc = raw_x * scale
            y_calc = raw_y * scale

            # B. APPLY MIRROR / FLIP (The Fix!)
            if swap_xy:
                x_calc, y_calc = y_calc, x_calc
            if mirror_x:
                x_calc = -x_calc
            if mirror_y:
                y_calc = -y_calc

            # C. APPLY ROTATION
            x_rot = x_calc * math.cos(angle_radians) - y_calc * math.sin(angle_radians)
            y_rot = x_calc * math.sin(angle_radians) + y_calc * math.cos(angle_radians)
            
            # D. APPLY TRANSLATION
            x_final = x_rot + dx
            y_final = y_rot + dy
            
            # --- Visual Markers ---
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE
            marker.id = i
            marker.action = Marker.ADD 
            marker.pose.position.x = x_final
            marker.pose.position.y = y_final
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker.lifetime = Duration(sec=1, nanosec=0)
            marker_array.markers.append(marker)

            # --- Robot Path ---
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x_final
            pose.pose.position.y = y_final
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.vis_path_pub.publish(marker_array)
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Waypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()