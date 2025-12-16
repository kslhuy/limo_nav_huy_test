import time
import rclpy
import math
import termios
import tty
import sys
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
from tf_transformations import quaternion_from_euler
from builtin_interfaces.msg import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import ReliabilityPolicy, QoSProfile

PI = 3.14159265

class PotentialField(Node):
    """
    A ROS2 node that implements a potential field algorithm for robot navigation.
    Attributes:
        mutuallyexclusive_group_1 (MutuallyExclusiveCallbackGroup): Callback group for the laser scan subscription.
        mutuallyexclusive_group_2 (MutuallyExclusiveCallbackGroup): Callback group for the main control loop timer.
        twist (Twist): Twist message for publishing velocity commands.
        cmd_pub (Publisher): Publisher for velocity commands.
        att_pub (Publisher): Publisher for attraction vector.
        rep_pub (Publisher): Publisher for repulsion vector.
        fin_pub (Publisher): Publisher for final vector.
        sub_scan (Subscription): Subscription for laser scan data.
        timer_period (float): Timer period for the main control loop.
        V_attraction (list): Attraction vector.
        attraction_vector (PoseStamped): PoseStamped message for the attraction vector.
        V_repulsion (list): Repulsion vector.
        repulsion_vector (PoseStamped): PoseStamped message for the repulsion vector.
        x_final (float): Final x component of the vector.
        y_final (float): Final y component of the vector.
        final_vector (PoseStamped): PoseStamped message for the final vector.
    Methods:
        __init__(): Initializes the PotentialField node.
        controller(): Main control loop that computes and publishes the final vector and velocity commands.
        create_vector_pose(x, y): Creates a PoseStamped message for a given vector.
        scan_callback(_msg): Callback function for processing laser scan data and updating the repulsion vector.
    """
    def __init__(self):
        super().__init__('potential_field_node')

        self.mutuallyexclusive_group_1 = MutuallyExclusiveCallbackGroup()
        self.mutuallyexclusive_group_2 = MutuallyExclusiveCallbackGroup()
        
        # Publishers
        self.twist = Twist()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.att_pub = self.create_publisher(PoseStamped, 'attraction_vector', 10)
        self.rep_pub = self.create_publisher(PoseStamped, 'repulsion_vector', 10)
        self.fin_pub = self.create_publisher(PoseStamped, 'final_vector', 10)
        
        # Create Subscriber for laser data
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.mutuallyexclusive_group_1)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10,
            callback_group=self.mutuallyexclusive_group_2
        )
        # Create Timer for main control loop
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.controller, self.mutuallyexclusive_group_2)

        
        # Create attraction and  repulsion vectors
        # self.V_attraction = [30.0, 0.0]
        # Real robot
        self.V_attraction = [15.0, 0.0]
        self.attraction_vector = self.create_vector_pose(self.V_attraction[0], self.V_attraction[1])
        self.V_repulsion = [0.0, 0.0]
        self.repulsion_vector = PoseStamped()
        self.navigation_active = False  # Flag to disable/enable controller
        self.manual_override = False  # Flag to disable potential field when pressing "P"

        # Stuck detection variables
        self.last_movement_time = time.time()
        self.stuck = False
        self.corner = False
        self.recovery_step = 0  # 0: Normal, 1: Moving Backward, 2: Turning
        
        # Start keyboard listener in a separate thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

    def goal_callback(self, msg):
        """ Callback when a navigation goal is set. """
        self.get_logger().info("Navigation goal received. Disabling potential field control.")
        self.navigation_active = True

    def keyboard_listener(self):
        """ Listen for keypress events and toggle Potential Field when 'P' is pressed. """
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())  # Set terminal to raw mode
            while True:
                key = sys.stdin.read(1).lower()  # Read a single keypress
                if key == 'p':
                    self.manual_override = not self.manual_override  # Toggle state
                    status = "DISABLED" if self.manual_override else "ENABLED"
                    self.get_logger().info(f"Manual override: Potential Field {status}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)  # Restore terminal settings

    def controller(self):
        """
        Controls the robot by computing the final vector from attraction and repulsion vectors,
        publishing these vectors to corresponding topics, and calculating the linear and angular
        velocities to command the robot's movement.
        The method performs the following steps:
        1. Computes the final vector by summing the attraction and repulsion vectors.
        2. Publishes the attraction, repulsion, and final vectors to their respective ROS topics.
        3. Calculates the linear velocity based on the magnitude of the final vector.
        4. Calculates the angular velocity based on the direction of the final vector.
        5. Publishes the computed velocities as a Twist message to command the robot.
        Attributes:
            self.x_final (float): X component of the final vector.
            self.y_final (float): Y component of the final vector.
            self.final_vector (Pose): Final vector as a Pose message.
            self.twist (Twist): Twist message containing linear and angular velocities.
        Publishes:
            self.att_pub (Publisher): Publisher for the attraction vector.
            self.rep_pub (Publisher): Publisher for the repulsion vector.
            self.fin_pub (Publisher): Publisher for the final vector.
            self.cmd_pub (Publisher): Publisher for the Twist message containing velocities.
        """
        """ Compute control commands only if Potential Field is active. """
        
        if self.navigation_active or self.manual_override:
            return  # Do nothing, let the navigation stack or manual mode take over

        # Check if the robot is stuck (not moving forward for 3 seconds)
        current_time = time.time()
        if abs(self.twist.linear.x)  > 0.02:  # If moving, update last_movement_time
            self.last_movement_time = current_time
            self.stuck = False
            self.recovery_step = 0  # Reset recovery steps

        elif (current_time - self.last_movement_time > 3.0) and self.corner:  # Stuck for 3s?
            self.stuck = True
            self.get_logger().warn("Robot is stuck! Initiating recovery sequence.")
            

        # Recovery behavior
        if self.stuck:
            if self.recovery_step == 0:  # Step 1: Move backward for 2 seconds
                self.twist.linear.x = -0.1
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                time.sleep(2)
                self.recovery_step = 1  # Move to next step
                self.last_movement_time = time.time()  # Reset movement timer
            elif self.recovery_step == 1:  # Step 2: Turn in place for 3 seconds
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.8  # Rotate in place
                self.cmd_pub.publish(self.twist)
                time.sleep(3)
                self.recovery_step = 0  # Try normal movement again
                self.last_movement_time = time.time()  # Reset movement timer
            return  # Skip normal movement when in recovery mode

        # Create final vector
        self.x_final = self.V_attraction[0] + self.V_repulsion[0]
        self.y_final = self.V_attraction[1] + self.V_repulsion[1]
        self.final_vector = self.create_vector_pose(self.x_final, self.y_final)
        
        # Publish all vectors to corresponding topics
        self.att_pub.publish(self.attraction_vector)
        self.rep_pub.publish(self.repulsion_vector)
        self.fin_pub.publish(self.final_vector)

         # Compute linear and angular velocities
        if self.x_final < 0.0:
            v_lin = 0.0
        else:
            v_lin = math.sqrt(math.pow(self.x_final,2) + math.pow(self.y_final,2))

        v_ang = math.atan2(self.y_final, self.x_final)
        # print("run")
        # self.twist.linear.x = v_lin / 250
        # self.twist.angular.z = v_ang / 4 * PI
        # Real robot
        # print(f"linear vel {v_lin / 3400} ")
        # print(f"angular vel {v_ang / 6 * PI} ")
        self.twist.linear.x = v_lin / 250
        self.twist.angular.z = v_ang / 6 * PI 


        self.cmd_pub.publish(self.twist)

    def create_vector_pose(self, x, y):
        """
        Creates a PoseStamped vector with the given x and y coordinates.
        Args:
            x (float): The x-coordinate for the pose.
            y (float): The y-coordinate for the pose.
        Returns:
            PoseStamped: A PoseStamped object with the specified position and orientation.
        """

        # Method to create a PoseStaamped vector
        vector = PoseStamped()
        vector.header.frame_id = "base_link"
        now = self.get_clock().now() 
        vector.header.stamp = Time(sec=int(now.nanoseconds // 1e9), nanosec=int(now.nanoseconds % 1e9))
        vector.pose.position.x = 0.0
        vector.pose.position.y = 0.0
        vector.pose.position.z = 0.0

        angle = math.atan2(y, x)
        q = quaternion_from_euler(0, 0, angle)
        quaternion_msg = Quaternion()
        quaternion_msg.x = q[0]
        quaternion_msg.y = q[1]
        quaternion_msg.z = q[2]
        quaternion_msg.w = q[3]
        vector.pose.orientation = quaternion_msg
        
        return vector

    def scan_callback(self, _msg):
        """
        Callback function for processing laser scan data.
        This function is triggered when a new laser scan message is received. It analyzes the laser scan data to compute a repulsion vector based on the detected obstacles. The repulsion vector is calculated by summing the inverse distances of obstacles within a certain range, weighted by their angles.
        Args:
            _msg (LaserScan): The laser scan message containing the range data and angle information.
        Attributes:
            V_repulsion (list): A list containing the x and y components of the repulsion vector.
            repulsion_vector (Pose): A Pose object representing the repulsion vector in the robot's coordinate frame.
        """

        # Analyze laser data and create the repulsion vector
        angle_min = _msg.angle_min
        angle_increment = _msg.angle_increment
        scan = _msg.ranges
        x_r = 0.0
        y_r = 0.0

        # Parameters to detect corner situations
        left_side = scan[len(scan) // 4]  # Approx. 90° left
        right_side = scan[3 * len(scan) // 4]  # Approx. 90° right
        front = scan[len(scan) // 2]  # Directly in front
        # self.get_logger().warn(left_side)
            
        # If both left and right have obstacles and front is blocked, assume corner
        if left_side < 0.5 and right_side < 0.5 and front < 0.4:
            self.get_logger().warn("Corner detected! Moving backward.")
            self.corner = True
            # self.twist.linear.x = -0.1
            # self.twist.angular.z = 0.0
            # self.cmd_pub.publish(self.twist)
            # return  # Skip normal repulsion calculation
        else:
            self.corner= False


        for i in range(len(scan)):
            if scan[i] < 0.6 and scan[i] > 0.08:
                x_r -= (1/scan[i])*math.cos(angle_min + angle_increment * i)
                y_r -= (1/scan[i])*math.sin(angle_min + angle_increment * i)

        self.V_repulsion = [x_r, y_r]

        self.repulsion_vector = self.create_vector_pose(self.V_repulsion[0], self.V_repulsion[1])

def main(args=None):
    rclpy.init(args=args)

    potential_field = PotentialField()

    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(potential_field)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        potential_field.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()