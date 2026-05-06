import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import sys

class LidarTestNode(Node):
    def __init__(self):
        super().__init__('lidar_test_node')
        
        # Subscribe to Lidar scan topic
        # Note: Depending on your bridge, the topic might be '/scan' or '/model/robot_minh/scan'
        # We start with '/scan' as it's the most common default
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
            
        self.get_logger().info('Lidar Test Node started. Listening on /scan')
        self.last_print_time = self.get_clock().now()

    def lidar_callback(self, msg):
        now = self.get_clock().now()
        # Print every 1 second to avoid flooding the terminal
        if (now - self.last_print_time).nanoseconds > 1e9:
            self.last_print_time = now
            
            # Filter out infinity/NaN for better visibility
            ranges = [round(r, 3) for r in msg.ranges]
            num_samples = len(msg.ranges)
            min_range = min(msg.ranges)
            max_range = max(msg.ranges)
            
            print(f"\n--- LIDAR DATA RECEIVED ---")
            print(f"Num Samples: {num_samples}")
            print(f"Min Range  : {min_range:.3f}m")
            print(f"Max Range  : {max_range:.3f}m")
            print(f"Ranges (first 12): {ranges[:12]}")
            sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = LidarTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
