import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from mecanum_ga_pkg.network import MecanumBrain

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.brain = MecanumBrain(input_size=12, hidden_size=16, output_size=2)
        
        qos_genome = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_lidar)
        self.genome_sub = self.create_subscription(Float32MultiArray, '/robot_genome', self.genome_callback, qos_genome)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.max_speed = 0.2 
        self.get_logger().info('robot_controller STARTED with max_speed=0.2')

    def genome_callback(self, msg):
        genome = np.array(msg.data)
        self.brain.set_genome(genome)

    def scan_callback(self, msg):
        if not hasattr(self.brain, 'weights_initialized') or not self.brain.weights_initialized:
            self.cmd_pub.publish(Twist())
            return

        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 2.0 
        input_data = ranges / 2.0 
        
        outputs = self.brain.forward(input_data)
        vx, vy = float(outputs[0]), float(outputs[1])
        
        cmd = Twist()
        cmd.linear.x = float(vx * self.max_speed)
        cmd.linear.y = float(vy * self.max_speed)
        self.cmd_pub.publish(cmd)
        
        self.get_logger().info(f"REAL SPEED -> x: {cmd.linear.x:.4f}", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
