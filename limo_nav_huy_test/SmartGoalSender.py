#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# We need this to "hear" the robot's confirmation
from geometry_msgs.msg import PoseWithCovarianceStamped as AmclPose

class SmartGoalSender(Node):
    def __init__(self):
        super().__init__('smart_goal_sender')
        
        # Publisher (The Mouth): Tells AMCL where to go
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
            
        # Subscriber (The Ear): Listens to where AMCL thinks it is
        self.current_amcl_pose = None
        self.amcl_sub = self.create_subscription(
            AmclPose, 
            '/amcl_pose', 
            self.amcl_pose_callback, 
            10)

        self.get_logger().info("Node started. Ready for commands.")

    def amcl_pose_callback(self, msg):
        """Constantly updates the robot's known position."""
        self.current_amcl_pose = msg

    def send_initial_pose_verified(self, x, y, yaw_deg):
        """
        Sends an initial pose and WAITS for AMCL to accept it.
        """
        self.get_logger().info(f"Attempting to set pose to x={x}, y={y}...")

        # 1. Prepare the message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
        msg.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
        
        # Set a small covariance (confidence)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.05 # X variance
        msg.pose.covariance[7] = 0.05 # Y variance
        msg.pose.covariance[35] = 0.05 # Yaw variance

        # 2. Safety Burst Loop (Send it multiple times)
        # We publish, then check, then publish again if needed.
        start_time = time.time()
        timeout = 5.0 # Give up after 5 seconds
        
        while (time.time() - start_time) < timeout:
            # A. Publish the guess
            self.initial_pose_pub.publish(msg)
            
            # B. Spin briefly so we can hear the callback from /amcl_pose
            rclpy.spin_once(self, timeout_sec=0.2)
            
            # C. Check if it worked
            if self.is_pose_match(x, y):
                self.get_logger().info("✅ SUCCESS: AMCL accepted the new pose!")
                return True
            
            self.get_logger().info("...waiting for AMCL to update...")

        self.get_logger().error("❌ FAILED: AMCL did not confirm the new pose. Is it running?")
        return False

    def is_pose_match(self, target_x, target_y, tolerance=0.15):
        """Check if current AMCL pose is close to target."""
        if self.current_amcl_pose is None:
            return False
        
        current_x = self.current_amcl_pose.pose.pose.position.x
        current_y = self.current_amcl_pose.pose.pose.position.y
        
        dist = math.sqrt((current_x - target_x)**2 + (current_y - target_y)**2)
        return dist < tolerance

    def interactive_loop(self):
        print("\n--- Interactive Mode ---")
        try:
            x = float(input("Enter X: "))
            y = float(input("Enter Y: "))
            yaw = float(input("Enter Yaw (deg): "))
            
            # Call the verified sender
            self.send_initial_pose_verified(x, y, yaw)
            
        except ValueError:
            print("Invalid number.")

def main():
    rclpy.init()
    node = SmartGoalSender()
    
    try:
        while rclpy.ok():
            node.interactive_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()