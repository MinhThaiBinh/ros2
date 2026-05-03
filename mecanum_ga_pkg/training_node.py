import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
import os
import subprocess
import time
from ament_index_python.packages import get_package_share_directory
from mecanum_ga_pkg.fitness_evaluator import FitnessEvaluator 
from mecanum_ga_pkg.ga_engine import GeneticAlgorithm
from std_msgs.msg import Float32MultiArray

class TrainingNode(Node):
    def __init__(self):
        super().__init__('training_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.population_size = 20
        self.ga = GeneticAlgorithm(population_size=self.population_size, genome_length=242)
        self.num_parallel = 5
        self.robots = []
        for i in range(self.num_parallel):
            robot_info = {
                'id': i, 'name': f'robot_{i}',
                'fitness_monitor': self.init_fitness_monitor(),
                'genome_pub': self.create_publisher(Float32MultiArray, f'/robot_{i}/genome', 10),
                'pose': None, 'vel': None, 'scan': None, 'is_active': False, 'individual_idx': -1
            }
            self.create_subscription(Pose, f'/model/robot_{i}/pose', lambda msg, r=robot_info: self.pose_cb(msg, r), 10)
            self.create_subscription(LaserScan, f'/world/congminh/model/robot_{i}/link/khung_xe/sensor/lidar/scan', lambda msg, r=robot_info: self.scan_cb(msg, r), 10)
            self.create_subscription(Twist, f'/model/robot_{i}/cmd_vel', lambda msg, r=robot_info: self.vel_cb(msg, r), 10)
            self.robots.append(robot_info)
        self.is_resetting = False
        self.timer = self.create_timer(0.1, self.update_loop)
        self.start_new_batch()

    def init_fitness_monitor(self):
        config_path = os.path.join(get_package_share_directory('mecanum_ga_pkg'), 'config', 'fitness_params.yaml')
        return FitnessEvaluator(config_path)

    def pose_cb(self, msg, robot): robot['pose'] = msg
    def scan_cb(self, msg, robot): robot['scan'] = msg.ranges
    def vel_cb(self, msg, robot): robot['vel'] = msg

    def start_new_batch(self):
        self.is_resetting = False
        self.start_time = self.get_clock().now()
        for i in range(self.num_parallel):
            if self.ga.current_idx < self.population_size:
                robot = self.robots[i]
                robot['individual_idx'] = self.ga.current_idx
                robot['is_active'] = True
                robot['fitness_monitor'].reset()
                genome = self.ga.get_next_genome()
                msg = Float32MultiArray(data=[float(x) for x in genome])
                robot['genome_pub'].publish(msg)
                self.get_logger().info(f"Bat dau Ind #{self.ga.current_idx + 1}")
                self.ga.current_idx += 1 

    def update_loop(self):
        if self.is_resetting: return
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed < 0: return
        for r in self.robots:
            if r['is_active'] and r['pose'] and r['vel'] and r['scan']:
                r['fitness_monitor'].update_metrics(r['pose'].position.x, r['pose'].position.y, r['vel'].linear.x, r['vel'].linear.y, r['scan'])
        if elapsed >= 30.0: self.finalize_batch()

    def finalize_batch(self):
        if self.is_resetting: return
        self.is_resetting = True
        for r in self.robots:
            if r['is_active']:
                score = r['fitness_monitor'].get_final_fitness()
                self.ga.save_fitness(score)
                r['is_active'] = False
        self.reset_simulation_parallel()
        self.timer_next = self.create_timer(2.0, self.auto_start_next)

    def auto_start_next(self):
        self.destroy_timer(self.timer_next)
        self.start_new_batch()

    def reset_simulation_parallel(self):
        for i in range(5):
            subprocess.run(["gz", "service", "-s", "/world/congminh/remove", "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean", "--timeout", "500", "--req", f'name: "robot_{i}", type: MODEL'], capture_output=True)
        subprocess.run(["gz", "service", "-s", "/world/congminh/control", "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", "--timeout", "500", "--req", "reset: {time_only: true}"], capture_output=True)
        time.sleep(1.0)
        for i in range(5):
            x_pos = (i - 2) * 0.5
            subprocess.run(["gz", "service", "-s", "/world/congminh/create", "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req", f'sdf_filename: "min_lidar_GA.sdf", name: "robot_{i}", pose: {{position: {{x: {x_pos}, y: 1.1, z: 0.04}}, orientation: {{x: 0, y: 0, z: -0.7071, w: 0.7071}}}}'], capture_output=True)

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
