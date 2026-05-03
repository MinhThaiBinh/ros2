import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from .network import MecanumBrain
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # 1. Khởi tạo Brain (Neural Network) với 12 đầu vào
        self.brain = MecanumBrain(input_size=12, hidden_size=16, output_size=2)
        
        # 2. Subscriber: Lấy dữ liệu thực từ Lidar
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
            
        # 3. Publisher: Gửi lệnh điều khiển robot
        self.cmd_pub = self.create_publisher(Twist, '/model/GA_8_TIA/cmd_vel', 10)
        
        self.get_logger().info('Robot Controller started. Working with REAL LIDAR data.')

    def lidar_callback(self, msg):
        # 4. Xử lý dữ liệu Lidar (12 giá trị)
        ranges = np.array(msg.ranges)
        
        # Xử lý các giá trị inf (ngoài tầm quét) thành 3.0m
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        ranges = np.where(np.isnan(ranges), msg.range_max, ranges)
        
        # 5. Đưa dữ liệu thực vào Neural Network
        outputs = self.brain.forward(ranges)
        
        # 6. Gửi lệnh điều khiển dựa trên output của não
        vx = float(outputs[0])
        vy = float(outputs[1])
        
        msg_twist = Twist()
        msg_twist.linear.x = vx * 0.3
        msg_twist.linear.y = vy * 0.3
        self.cmd_pub.publish(msg_twist)
        
        # Log kết quả mỗi 1 giây
        self.get_logger().info(f'Real Lidar -> Vx: {vx:.2f}, Vy: {vy:.2f}', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
