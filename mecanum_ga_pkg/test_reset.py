import rclpy
from rclpy.node import Node
import subprocess
import time
from geometry_msgs.msg import Twist

class ManualResetTest(Node):
    def __init__(self):
        super().__init__('manual_reset_test')
        self.get_logger().info('\n=== MANUAL RESET TEST NODE ===')
        self.get_logger().info('Nhan Ctrl+C de thuc hien Reset thu nghiem...')

    def perform_reset(self):
        self.get_logger().info('Dang thuc hien quy trinh reset...')
        
        try:
            # 1. Dung robot
            self.get_logger().info('1. Gui lenh dung robot...')
            stop_pub = self.create_publisher(Twist, '/model/GA_8_TIA/cmd_vel', 10)
            stop_msg = Twist()
            stop_pub.publish(stop_msg)
            time.sleep(0.5)

            # 2. Xóa robot cũ
            self.get_logger().info('2. Xoa robot GA_8_TIA cu...')
            subprocess.run(["gz", "service", "-s", "/world/congminh/remove", 
                          "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean", 
                          "--timeout", "500", "--req", 'name: "GA_8_TIA", type: MODEL'], capture_output=True)

            # 3. Reset Time
            self.get_logger().info('3. Reset thoi gian (time_only)...')
            res_time = subprocess.run(["gz", "service", "-s", "/world/congminh/control", 
                          "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", 
                          "--timeout", "1000", "--req", "reset: {time_only: true}"], capture_output=True, text=True)
            self.get_logger().info(f'Ket qua Reset Time: {res_time.stdout or res_time.stderr}')

            time.sleep(0.5)

            # 4. Spawn lại robot mới
            self.get_logger().info('4. Spawn lai robot GA_8_TIA moi...')
            # Chỉnh Z=0.04 để robot sát mặt sàn
            res_spawn = subprocess.run(["gz", "service", "-s", "/world/congminh/create",
                          "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
                          "--timeout", "1000", 
                          "--req", 'sdf_filename: "minh_lidar_GA.sdf", name: "GA_8_TIA", pose: {position: {x: 0, y: 1.1, z: 0.04}, orientation: {x: 0, y: 0, z: -0.7071, w: 0.7071}}'],
                          capture_output=True, text=True)
            self.get_logger().info(f'Ket qua Spawn: {res_spawn.stdout or res_spawn.stderr}')

            time.sleep(1.0)
            
            # 5. Kiem tra vi tri hien tai
            self.get_logger().info('5. Kiem tra toa do thuc te...')
            res_check = subprocess.run(["gz", "model", "-m", "GA_8_TIA", "-p"], capture_output=True, text=True)
            self.get_logger().info(f'Toa do hien tai:\n{res_check.stdout}')

            self.get_logger().info('=== RESET HOAN TAT ===')
            
        except Exception as e:
            self.get_logger().error(f'Loi trong qua trinh reset: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ManualResetTest()
    
    try:
        # Thay vi doi Ctrl+C, chung ta se cho user vao rclpy.spin va co the goi thu cong sau
        node.get_logger().info('Tu dong thuc hien reset sau 2 giay...')
        time.sleep(2.0)
        node.perform_reset()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
