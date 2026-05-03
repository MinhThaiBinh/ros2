import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class MecanumBrain:
    def __init__(self, input_size=12, hidden_size=16, output_size=2):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        
        # Khởi tạo trọng số ngẫu nhiên [-1, 1]
        self.w1 = np.random.uniform(-1, 1, (input_size, hidden_size))
        self.b1 = np.zeros(hidden_size)
        self.w2 = np.random.uniform(-1, 1, (hidden_size, output_size))
        self.b2 = np.zeros(output_size)

    def forward(self, inputs):
        # Lớp ẩn
        h = np.tanh(np.dot(inputs, self.w1) + self.b1)
        # Đầu ra: Vx, Vy (trong khoảng -1 đến 1)
        outputs = np.tanh(np.dot(h, self.w2) + self.b2)
        return outputs # Trả về mảng [vx, vy]

    def get_genome(self):
        return np.concatenate([
            self.w1.flatten(), 
            self.b1.flatten(), 
            self.w2.flatten(), 
            self.b2.flatten()
        ])

    def set_genome(self, genome):
        w1_end = self.input_size * self.hidden_size
        b1_end = w1_end + self.hidden_size
        w2_end = b1_end + (self.hidden_size * self.output_size)
        
        self.w1 = genome[0:w1_end].reshape(self.input_size, self.hidden_size)
        self.b1 = genome[w1_end:b1_end]
        self.w2 = genome[b1_end:w2_end].reshape(self.hidden_size, self.output_size)
        self.b2 = genome[w2_end:]

class NetworkTestNode(Node):
    def __init__(self):
        super().__init__('network_test_node')
        self.brain = MecanumBrain(input_size=12, output_size=2)
        
        # Publisher để điều khiển robot trong mô phỏng
        self.cmd_pub = self.create_publisher(Twist, '/model/GA_8_TIA/cmd_vel', 10)
        
        # Timer chạy mỗi 0.1s để giả lập việc xử lý
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Network Test Node started with MOCK DATA')

    def timer_callback(self):
        # 1. Tạo dữ liệu giả lập (Mock Lidar Data) - 12 giá trị ngẫu nhiên từ 0.0 đến 3.0
        mock_lidar = np.random.uniform(0.0, 3.0, 12)
        
        # 2. Chạy qua mạng Neural
        outputs = self.brain.forward(mock_lidar)
        vx = float(outputs[0])
        vy = float(outputs[1])
        
        # 3. Gửi lệnh Twist
        msg = Twist()
        msg.linear.x = vx * 0.1  # Giới hạn tốc độ tối đa 0.2m/s
        msg.linear.y = vy * 0.1
        self.cmd_pub.publish(msg)
        
        # 4. Log kết quả
        self.get_logger().info(f'MOCK Input (mean): {np.mean(mock_lidar):.2f} | Output -> Vx: {vx:.2f}, Vy: {vy:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = NetworkTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Dừng robot khi tắt node
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
