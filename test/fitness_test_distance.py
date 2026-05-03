import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose
import math
import sys

class FitnessTestNode(Node):
    def __init__(self):
        super().__init__('fitness_test_node')
        
        # Subscribe to robot pose (Ground Truth from direct Pose bridge)
        self.subscription = self.create_subscription(
            Pose,
            '/model/GA_8_TIA/pose',
            self.direct_pose_callback,
            10)
            
        # Keep world info fallback as TFMessage
        self.world_sub = self.create_subscription(
            TFMessage,
            '/world/congminh/pose/info',
            self.tf_callback,
            10)
            
        self.start_x = None
        self.start_y = None
        self.last_x = 0.0
        self.last_y = 0.0
        self.total_dist = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_log_time = 0.0 # Track last log time
        
        self.get_logger().info('Fitness Test Node started. Listening on /model/GA_8_TIA/pose (Pose) and /world/congminh/pose/info (TF)')

    def direct_pose_callback(self, msg):
        # Ép log ngay lập tức để confirm nhận được data
        # print(f"DEBUG: Data received from Pose: {msg.position.x:.4f}")
        # sys.stdout.flush()
        self.process_position(msg.position.x, msg.position.y, "DIRECT")

    def tf_callback(self, msg):
        for transform in msg.transforms:
            child = transform.child_frame_id
            translation = transform.transform.translation
            
            # Khung xe là frame chính xác nhất từ SDF
            if child == 'khung_xe':
                self.process_position(translation.x, translation.y, "TF_BODY")
            # Fallback nếu bridge dùng tên model
            elif child == 'robot_minh' or child == 'GA_8_TIA':
                self.process_position(translation.x, translation.y, "TF_ROBOT")
            elif 'lidar' in child:
                self.process_position(translation.x, translation.y, "TF_LIDAR")

    def process_position(self, x, y, source):
        # Sau khi sửa SDF publish_link_pose=false, DIRECT topic chỉ còn 1 pose duy nhất của model
        if source == "DIRECT":
            pass 

        self.current_x = x
        self.current_y = y
        
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.last_x = x
            self.last_y = y
            self.total_dist = 0.0
            print(f'--- STARTING POSITION SET ({source}) ---')
            sys.stdout.flush()
            return

        # Tính khoảng cách di chuyển thực tế
        delta_d = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)

        # Ngưỡng lọc rung (chỉ lấy chuyển động > 2mm)
        if delta_d > 0.002:
            self.total_dist += delta_d
            self.last_x = x
            self.last_y = y

        # Log định kỳ (0.5s)
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_log_time >= 0.5:
            print(f'[{source}] POS: ({x:.4f}, {y:.4f}) | TOTAL_DIST: {self.total_dist:.4f}m')
            sys.stdout.flush()
            self.last_log_time = current_time
        
def main(args=None):
    rclpy.init(args=args)
    node = FitnessTestNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Node Error: {e}", file=sys.stderr)
    finally:
        # Standard safety check for rclpy shutdown
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
