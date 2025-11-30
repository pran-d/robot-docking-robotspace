import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math

class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')
        self.get_logger().info("Docking Node Started")
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_pose',
            self.pose_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_distance = 0.25 # Stop 0.25m away to be safe
        self.last_msg_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.control_loop)
        self.current_pose = None

    def pose_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.current_pose = msg

    def control_loop(self):
        # Check if we have lost the marker
        time_since_last_msg = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        
        if time_since_last_msg > 1.0:
            # Stop if marker is lost
            self.publisher_.publish(Twist())
            # self.get_logger().warn("Marker lost! Stopping.")
            return

        if self.current_pose is None:
            return

        # Pose is in camera frame (optical).
        # Z is forward distance.
        # X is horizontal offset (right).
        
        x = self.current_pose.pose.position.x
        z = self.current_pose.pose.position.z
        
        distance = z
        # Angle to the marker center
        angle_error = math.atan2(x, z)
        
        cmd = Twist()
        
        # Constants
        k_linear = 0.4
        k_angular = 1.5
        min_linear_speed = 0.02
        angle_tolerance = 0.02 # radians
        eps = 0.005
        
        if distance > self.target_distance + eps:
            # Move towards the marker
            linear_speed = k_linear * (distance - self.target_distance)
            
            # Ensure minimum speed to overcome friction/deadband, but only if we are far enough
            if linear_speed < min_linear_speed and linear_speed > 0.005:
                linear_speed = min_linear_speed
                
            cmd.linear.x = linear_speed
            cmd.angular.z = -k_angular * angle_error
            
            self.get_logger().info(f'Approaching... Dist: {distance:.2f}, Angle: {angle_error:.2f}, Cmd: [{cmd.linear.x:.2f}, {cmd.angular.z:.2f}]')
            
        elif abs(angle_error) > angle_tolerance:
            # Reached distance, but need to rotate to face the marker
            cmd.linear.x = 0.0
            cmd.angular.z = -k_angular * angle_error
            self.get_logger().info(f'Aligning... Angle Error: {angle_error:.2f}')
            
        else:
            # Docked and aligned
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Docked!')
            
        # Limit speeds
        cmd.linear.x = max(min(cmd.linear.x, 0.5), -0.5)
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)
            
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
