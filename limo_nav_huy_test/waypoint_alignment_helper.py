#!/usr/bin/env python3
"""
Interactive Waypoint Alignment Helper

This tool helps you find the right transformation parameters to align
SDCSRoadMap waypoints with your physical map.

Usage:
1. Run: ros2 run limo_nav_huy_test waypoint_alignment_helper
2. Open RViz and add:
   - /map topic (your physical map)
   - /waypoints_test (MarkerArray - the waypoints you're adjusting)
3. Use this terminal to adjust parameters interactively
4. When aligned, copy the final parameters to your launch file
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from hal.products.mats import SDCSRoadMap
import numpy as np
import sys
import select
import tty
import termios


class WaypointAlignmentHelper(Node):
    def __init__(self):
        super().__init__('waypoint_alignment_helper')
        
        # Current transformation parameters
        self.translation_x = 0.0
        self.translation_y = 0.0
        self.rotation_deg = 90.0
        self.scale = 1.0
        
        # SDCSRoadMap setup
        self.nodeSequence = [10, 2, 4, 6, 8, 10]
        self.roadmap = SDCSRoadMap(leftHandTraffic=False, useSmallMap=True)
        self.waypointSequence = self.roadmap.generate_path(self.nodeSequence) * 0.975
        self.waypoints_x = self.waypointSequence[0]
        self.waypoints_y = self.waypointSequence[1]
        
        # Publishers
        self.vis_pub = self.create_publisher(MarkerArray, "/waypoints_test", 10)
        self.path_pub = self.create_publisher(Path, "/path_test", 10)
        
        # Timer for continuous publishing
        self.timer = self.create_timer(0.5, self.publish_waypoints)
        
        self.get_logger().info("="*70)
        self.get_logger().info("Waypoint Alignment Helper Started")
        self.get_logger().info("="*70)
        self.print_instructions()
        self.print_current_params()
        
    def print_instructions(self):
        print("\n" + "="*70)
        print("INTERACTIVE CONTROLS:")
        print("="*70)
        print("Translation (move waypoints):")
        print("  w/s : Move Y +/- 0.1m")
        print("  a/d : Move X -/+ 0.1m")
        print("  W/S : Move Y +/- 0.5m")
        print("  A/D : Move X -/+ 0.5m")
        print()
        print("Rotation (rotate waypoints):")
        print("  q/e : Rotate -/+ 5°")
        print("  Q/E : Rotate -/+ 15°")
        print()
        print("Scale (resize waypoints):")
        print("  z/x : Scale -/+ 0.05")
        print("  Z/X : Scale -/+ 0.1")
        print()
        print("Utility:")
        print("  r   : Reset to defaults")
        print("  p   : Print current parameters")
        print("  c   : Copy parameters for launch file")
        print("  ESC : Quit")
        print("="*70)
        print("\nOpen RViz and add:")
        print("  1. /map topic (your physical map)")
        print("  2. /waypoints_test (MarkerArray, frame: map_rotated)")
        print("  3. Set Fixed Frame: map or map_rotated")
        print("="*70 + "\n")
    
    def print_current_params(self):
        print(f"\n[Current Parameters]")
        print(f"  translation_offset: [{self.translation_x:.3f}, {self.translation_y:.3f}]")
        print(f"  rotation_offset: {self.rotation_deg:.1f}°")
        print(f"  scale: {self.scale:.3f}")
        print()
    
    def print_launch_params(self):
        print("\n" + "="*70)
        print("COPY THESE PARAMETERS TO YOUR LAUNCH FILE:")
        print("="*70)
        print("parameters=[{")
        print(f"    'translation_offset': [{self.translation_x:.3f}, {self.translation_y:.3f}],")
        print(f"    'rotation_offset': {self.rotation_deg:.1f},")
        print(f"    'scale': {self.scale:.3f}")
        print("}]")
        print("="*70 + "\n")
    
    def reset_params(self):
        self.translation_x = 0.0
        self.translation_y = 0.0
        self.rotation_deg = 90.0
        self.scale = 1.0
        self.get_logger().info("Parameters reset to defaults")
        self.print_current_params()
    
    def publish_waypoints(self):
        """Publish transformed waypoints"""
        marker_array = MarkerArray()
        path_msg = Path()
        path_msg.header.frame_id = "map_rotated"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, (raw_x, raw_y) in enumerate(zip(self.waypoints_x, self.waypoints_y)):
            # Apply QCar transformation pipeline
            # 1. Scale
            wp_scaled = np.array([raw_x * self.scale, raw_y * self.scale])
            
            # 2. Translate
            t = np.array([self.translation_x, self.translation_y])
            wp_translated = wp_scaled + t
            
            # 3. Rotate (negative angle, QCar style)
            angle_rad = -self.rotation_deg * np.pi / 180.0
            R_matrix = np.array([
                [np.cos(angle_rad), -np.sin(angle_rad)],
                [np.sin(angle_rad),  np.cos(angle_rad)]
            ])
            wp_final = wp_translated @ R_matrix
            
            x_final = wp_final[0]
            y_final = wp_final[1]
            
            # Marker
            marker = Marker()
            marker.header.frame_id = "map_rotated"
            marker.type = Marker.SPHERE
            marker.id = i
            marker.action = Marker.ADD
            marker.pose.position.x = x_final
            marker.pose.position.y = y_final
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.lifetime = Duration(sec=1, nanosec=0)
            marker_array.markers.append(marker)
            
            # Path
            pose = PoseStamped()
            pose.header.frame_id = "map_rotated"
            pose.pose.position.x = x_final
            pose.pose.position.y = y_final
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.vis_pub.publish(marker_array)
        self.path_pub.publish(path_msg)


def get_key():
    """Get single keypress (non-blocking)"""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = WaypointAlignmentHelper()
    
    # Set terminal to raw mode for key capture
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            key = get_key()
            if key:
                changed = True
                
                # Translation
                if key == 'w':
                    node.translation_y += 0.1
                elif key == 's':
                    node.translation_y -= 0.1
                elif key == 'a':
                    node.translation_x -= 0.1
                elif key == 'd':
                    node.translation_x += 0.1
                elif key == 'W':
                    node.translation_y += 0.5
                elif key == 'S':
                    node.translation_y -= 0.5
                elif key == 'A':
                    node.translation_x -= 0.5
                elif key == 'D':
                    node.translation_x += 0.5
                
                # Rotation
                elif key == 'q':
                    node.rotation_deg -= 5.0
                elif key == 'e':
                    node.rotation_deg += 5.0
                elif key == 'Q':
                    node.rotation_deg -= 15.0
                elif key == 'E':
                    node.rotation_deg += 15.0
                
                # Scale
                elif key == 'z':
                    node.scale = max(0.1, node.scale - 0.05)
                elif key == 'x':
                    node.scale += 0.05
                elif key == 'Z':
                    node.scale = max(0.1, node.scale - 0.1)
                elif key == 'X':
                    node.scale += 0.1
                
                # Utility
                elif key == 'r':
                    node.reset_params()
                    changed = False
                elif key == 'p':
                    node.print_current_params()
                    changed = False
                elif key == 'c':
                    node.print_launch_params()
                    changed = False
                elif key == '\x1b':  # ESC
                    print("\nExiting...")
                    break
                else:
                    changed = False
                
                if changed:
                    node.print_current_params()
    
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
