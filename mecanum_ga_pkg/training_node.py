import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, PoseArray
import os
import subprocess
import time
from ament_index_python.packages import get_package_share_directory
from mecanum_ga_pkg.fitness_evaluator import FitnessEvaluator 
from mecanum_ga_pkg.ga_engine import GeneticAlgorithm
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray

class TrainingNode(Node):
    def __init__(self):
        super().__init__('training_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.population_size = 20
        self.ga = GeneticAlgorithm(population_size=self.population_size, genome_length=242)
        self.num_parallel = 1
        self.robots = []
        
        # QoS Genome Publisher: Chế độ TRANSIENT_LOCAL (giữ giá trị cuối cho subscriber mới)
        qos_genome = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS Lidar: Đổi sang RELIABLE để khớp với Bridge
        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        for i in range(self.num_parallel):
            r_name = 'GA_8_TIA' 
            robot_info = {
                'id': i, 
                'name': r_name,
                'fitness_monitor': self.init_fitness_monitor(),
                'genome_pub': self.create_publisher(Float32MultiArray, '/robot_genome', qos_genome),
                'pose': None, 
                'vel': None, 
                'scan': None, 
                'is_active': False, 
                'individual_idx': -1,
                'spawn_y': 1.1 
            }
        # Subscribe riêng cho từng robot
        self.create_subscription(PoseArray, f'/model/{r_name}/pose', lambda msg, r=robot_info: self.pose_cb(msg, r), 10)
        self.create_subscription(LaserScan, '/scan', lambda msg, r=robot_info: self.scan_cb(msg, r), qos_lidar)
        self.create_subscription(Twist, '/cmd_vel', lambda msg, r=robot_info: self.vel_cb(msg, r), 10)
        
        # Subscriber cho ket qua danh gia tu node khac (neu co)
        # self.result_sub = self.create_subscription(Float32, '/evaluation_result', self.result_callback, 10)
        
        self.robots.append(robot_info)
            
        self.is_resetting = False
        self.timer = self.create_timer(0.01, self.update_loop)
        self.start_new_batch()

    def init_fitness_monitor(self):
        config_path = os.path.join(get_package_share_directory('mecanum_ga_pkg'), 'config', 'fitness_params.yaml')
        return FitnessEvaluator(config_path)

    def pose_cb(self, msg, robot): 
        if msg.poses:
            robot['pose'] = msg.poses[0]
    def scan_cb(self, msg, robot): robot['scan'] = msg.ranges
    def vel_cb(self, msg, robot): robot['vel'] = msg

    def reset_simulation_parallel(self):
        # Reset robot về vị trí y=1.0 để cách xa tường h1 (y=1.2), z=0.03 để tránh rơi tự do
        reset_cmd = [
            'gz', 'service', '-s', '/world/congminh/set_pose',
            '--reqtype', 'gz.msgs.EntityPose', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', 'entity: {name: "GA_8_TIA", type: MODEL}, pose: {position: {x: 0, y: 1.0, z: 0.03}, orientation: {x: 0, y: 0, z: -0.7071, w: 0.7071}}'
        ]
        try:
            subprocess.run(reset_cmd, check=True, capture_output=True)
        except Exception as e:
            self.get_logger().error(f"Failed to reset pose: {e}")

    def start_new_batch(self):
        self.is_resetting = False
        self.start_time = self.get_clock().now()
        
        # Chỉ lấy robot đầu tiên vì num_parallel = 1
        robot = self.robots[0]
        
        if self.ga.current_idx < self.population_size:
            robot['individual_idx'] = self.ga.current_idx
            robot['is_active'] = True
            robot['fitness_monitor'].reset()
            
            genome = self.ga.get_next_genome()
            msg = Float32MultiArray(data=[float(x) for x in genome])
            
            self.get_logger().info(f"Sending genome to {robot['name']} (Individual #{self.ga.current_idx + 1})...")
            # Gửi genome
            for _ in range(5):
                robot['genome_pub'].publish(msg)
                time.sleep(0.1)
            
            self.get_logger().info(f"--- THE HE #{self.ga.generation} | DANH GIA #{self.ga.current_idx + 1}/{self.population_size} ---")
            self.ga.current_idx += 1
        else:
            self.get_logger().info("--- DA XONG MOT THE HE! DANG TIEN HOA... ---")
            self.ga.evolve()
            self.ga.current_idx = 0
            # Sau khi evolve xong, gọi lại để bắt đầu cá thể đầu tiên của thế hệ mới
            self.start_new_batch()

    def update_loop(self):
        if self.is_resetting: return
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed < 0: return

        # Dot nay ket thuc khi CO IT NHAT MOT robot va cham hoac het gio
        # (Trong tuong lai co the cai tien de moi robot chay doc lap)
        should_finalize = False
        if elapsed >= 30.0:
            should_finalize = True

        for r in self.robots:
            if r['is_active']:
                # Kiem tra neu robot vang ra khoi me cung (gia su me cung nam trong khoang x:[-5,5], y:[-1,10])
                # Tuy vao kich thuoc me cung thuc te trong mecung.sdf, ban co the dieu chinh con so nay
                if r['pose']:
                    pos = r['pose'].position
                    if abs(pos.x) > 10.0 or pos.y < -2.0 or pos.y > 15.0:
                        self.get_logger().error(f"Robot {r['name']} vang ra khoi me cung tại ({pos.x:.2f}, {pos.y:.2f})! Reset ngay.")
                        should_finalize = True

                # Log định kỳ mỗi 5 giây (0.01 * 500)
                if r['pose'] and r['vel'] and r['scan']:
                    # Tinh toan toa do tuong doi trong me cung cua no (tru di spawn_y)
                    rel_y = r['pose'].position.y - r['spawn_y']
                    r['fitness_monitor'].update_metrics(r['pose'].position.x, rel_y, r['vel'].linear.x, r['vel'].linear.y, r['scan'])
                    
                    if int(elapsed * 100) % 500 == 0:
                        cur_fit = r['fitness_monitor'].get_final_fitness()
                        self.get_logger().info(f"Robot {r['name']} | Time: {elapsed:.1f}s | Current Fitness: {cur_fit:.4f}")

                    if r['fitness_monitor'].collision_detected:
                        self.get_logger().warn(f"Robot {r['name']} va cham!")
                        should_finalize = True
                else:
                    # Log để biết con nào thiếu data
                    if elapsed > 3.0: # Đợi lâu hơn một chút
                        missing = []
                        if not r['pose']: missing.append("pose")
                        if not r['vel']: missing.append("vel")
                        if not r['scan']: missing.append("scan")
                        if missing:
                            self.get_logger().info(f"Robot {r['name']} thieu data: {missing}", throttle_duration_sec=5.0)
        
        if should_finalize:
            self.finalize_batch()

    def finalize_batch(self):
        if self.is_resetting: return
        self.is_resetting = True
        
        # Dung cac lenh dieu khien ngay lap tuc
        stop_msg = Twist()
        for r in self.robots:
            stop_pub = self.create_publisher(Twist, f'/model/{r["name"]}/cmd_vel', 10)
            stop_pub.publish(stop_msg)

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        for r in self.robots:
            if r['is_active']:
                score = r['fitness_monitor'].get_final_fitness()
                self.get_logger().info(f"==> Robot {r['name']} ket thuc | Fitness: {score:.2f} | Time: {elapsed:.1f}s")
                self.ga.save_fitness(score)
                r['is_active'] = False
                
        # Áp dụng cơ chế Remove & Spawn lại từ test_reset.py để đảm bảo vị trí tuyệt đối
        self.perform_full_reset()
        
        # Chỉ tạo timer nếu chưa có, để tránh chồng chéo nhiều timer
        if hasattr(self, 'timer_next') and self.timer_next is not None:
            self.timer_next.destroy()
        self.timer_next = self.create_timer(1.0, self.start_next_indiv_callback)

    def start_next_indiv_callback(self):
        """Callback để chuyển sang cá thể tiếp theo một lần duy nhất"""
        self.timer_next.destroy()
        self.timer_next = None
        self.start_new_batch()

    def perform_full_reset(self):
        self.get_logger().info('Dang thuc hien quy trinh reset giong node test_reset (Delete & Spawn)...')
        
        try:
            # 1. Dừng robot và reset data cache
            stop_msg = Twist()
            for r in self.robots:
                stop_pub = self.create_publisher(Twist, f'/model/{r["name"]}/cmd_vel', 10)
                stop_pub.publish(stop_msg)
                r['pose'] = None
                r['vel'] = None
                r['scan'] = None
            time.sleep(0.5)

            # 2. Xóa robot cũ
            self.get_logger().info('2. Xoa robot GA_8_TIA cu...')
            subprocess.run(["gz", "service", "-s", "/world/congminh/remove", 
                          "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean", 
                          "--timeout", "1000", "--req", 'name: "GA_8_TIA", type: MODEL'], capture_output=True)

            # 3. Reset Time
            self.get_logger().info('3. Reset thoi gian (time_only)...')
            subprocess.run(["gz", "service", "-s", "/world/congminh/control", 
                          "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", 
                          "--timeout", "1000", "--req", "reset: {time_only: true}"], capture_output=True)
            time.sleep(0.5)

            # 4. Spawn lại robot mới (Vùng an toàn: x:0, y:1.1, z:0.04)
            self.get_logger().info('4. Spawn lai robot GA_8_TIA moi...')
            subprocess.run(["gz", "service", "-s", "/world/congminh/create",
                          "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
                          "--timeout", "2000", 
                          "--req", 'sdf_filename: "minh_lidar_GA.sdf", name: "GA_8_TIA", pose: {position: {x: 0, y: 1.1, z: 0.04}, orientation: {x: 0, y: 0, z: -0.7071, w: 0.7071}}'],
                          capture_output=True)

            time.sleep(1.0)
            self.get_logger().info('=== RESET HOAN TAT ===')
            
        except Exception as e:
            self.get_logger().error(f'Loi trong qua trinh reset: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TrainingNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
