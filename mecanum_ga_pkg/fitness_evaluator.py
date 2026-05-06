import yaml
import math
import numpy as np
import time

class FitnessEvaluator:
    def __init__(self, config_path='config/fitness_params.yaml'):
        # Nạp cấu hình từ file YAML
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            
            self.w_dist = float(config['fitness']['w_dist'])
            self.w_collision = float(config['fitness']['w_collision'])
            self.w_oscillation = float(config['fitness']['w_oscillation'])
        except Exception as e:
            print(f"Error loading config: {e}")
            # Giá trị mặc định nếu lỗi load file
            self.w_dist = 1.0
            self.w_collision = 50.0
            self.w_oscillation = 5.0
        
        # State để theo dỗi (tích lũy trong suốt quá trình chạy của 1 cá thể)
        self.total_dist = 0.0
        self.collision_detected = False
        self.last_pose = None  # (x, y)
        self.last_vel = (0.0, 0.0) # (vx, vy)
        self.stability_sum = 0.0

    def update_metrics(self, current_x, current_y, current_vx, current_vy, lidar_ranges):
        """
        Cập nhật các chỉ số dựa trên dữ liệu từ ROS 2 callback
        """
        # 1. Tính quãng đường (Lọc nhiễu 2mm)
        if self.last_pose:
            d = math.sqrt((current_x - self.last_pose[0])**2 + (current_y - self.last_pose[1])**2)
            if d > 0.002:
                self.total_dist += d
        self.last_pose = (current_x, current_y)

        # 2. Kiểm tra Va chạm (Lidar)
        # Lọc bỏ các giá trị <= 0 (lỗi) và nan
        valid_ranges = [r for r in lidar_ranges if r > 0 and not np.isnan(r)]
        
        if len(valid_ranges) > 0:
            min_dist = min(valid_ranges)
            # Log mỗi 1 giây để tránh tràn log
            if time.time() - getattr(self, '_last_log_time', 0) > 1.0:
                print(f"DEBUG: Lidar Min Dist: {min_dist:.3f}m")
                self._last_log_time = time.time()

            # Ngưỡng va chạm: 0.18m
            if min_dist < 0.18:
                self.collision_detected = True
                print(f"!!! COLLISION DETECTED !!! Min Dist: {min_dist:.3f}m")

        # 3. Tính Rung lắc (Stability) - Tích lũy sự thay đổi vận tốc
        osc = abs(current_vx - self.last_vel[0]) + abs(current_vy - self.last_vel[1])
        self.stability_sum += osc
        self.last_vel = (current_vx, current_vy)

    def get_final_fitness(self):
        """
        Trả về điểm fitness tổng kết sau khi kết thúc 1 lần chạy
        """
        score = (self.w_dist * self.total_dist)                 - (self.w_collision * int(self.collision_detected))                 - (self.w_oscillation * self.stability_sum)
        
        return max(0.0, float(score))

    def reset(self):
        """Reset state cho cá thể mới"""
        self.total_dist = 0.0
        self.collision_detected = False
        self.last_pose = None
        self.last_vel = (0.0, 0.0)
        self.stability_sum = 0.0
