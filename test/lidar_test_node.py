import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarIndexTest(Node):
    def __init__(self):
        super().__init__('lidar_index_test')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.get_logger().info('--- Đang đợi dữ liệu Lidar... ---')
        self.first_run = True

    def listener_callback(self, msg):
        if self.first_run:
            num_beams = len(msg.ranges)
            self.get_logger().info(f'Số lượng tia: {num_beams}')
            self.get_logger().info(f'Góc bắt đầu: {msg.angle_min:.2f} rad ({math.degrees(msg.angle_min):.1f}°)')
            self.get_logger().info(f'Góc kết thúc: {msg.angle_max:.2f} rad ({math.degrees(msg.angle_max):.1f}°)')
            self.get_logger().info(f'Bước nhảy góc: {msg.angle_increment:.4f} rad')
            self.get_logger().info('--------------------------------------------')
            self.get_logger().info('Dưới đây là vị trí các tia (Index -> Góc):')
            
            for i in range(num_beams):
                angle_rad = msg.angle_min + (i * msg.angle_increment)
                angle_deg = math.degrees(angle_rad)
                # Chuẩn hóa góc về -180 đến 180 để dễ đọc
                if angle_deg > 180: angle_deg -= 360
                
                # Xác định hướng tương đối
                direction = ""
                if abs(angle_deg) < 15: direction = "[TRƯỚC MẶT]"
                elif 85 < angle_deg < 95: direction = "[BÊN TRÁI]"
                elif -95 < angle_deg < -85: direction = "[BÊN PHẢI]"
                elif abs(angle_deg) > 165: direction = "[PHÍA SAU]"

                self.get_logger().info(f'Index {i:2d}: {angle_deg:6.1f}° {direction}')
            
            self.get_logger().info('--------------------------------------------')
            self.get_logger().info('Hệ thống sẽ in giá trị khoảng cách tia 0 (Trước mặt) mỗi 2 giây:')
            self.first_run = False

        # In giá trị tia phía trước để test
        self.get_logger().info(f'Khoảng cách tia [0]: {msg.ranges[0]:.2f}m', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = LidarIndexTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
