# Hướng dẫn Kiểm tra và Huấn luyện (Cập nhật 03/05/2026)

Tài liệu này hướng dẫn cách khởi chạy hệ thống Simulation Gazebo với giao diện đồ họa (GUI) và kiểm tra dữ liệu cảm biến Lidar cũng như Fitness trong ROS 2 Jazzy.

---

## 1. Khởi chạy Hệ thống Simulation (Gazebo + GUI)

Sử dụng lệnh launch đã được tối ưu hóa để tự động dọn dẹp các tiến trình cũ, mở giao diện Gazebo, spawn robot tại tọa độ `(0, 1.1, 0.1)` và thiết lập Bridge.

```bash
# BƯỚC 1: Dọn dẹp các tiến trình cũ (Gazebo, Bridge)
pkill -9 -f "gz sim|parameter_bridge|ruby" && sleep 2

# BƯỚC 2: Build và Source môi trường
cd ~/ros2_ws
colcon build --packages-select mecanum_ga_pkg --symlink-install
source install/setup.bash

# BƯỚC 3: Khởi chạy file launch (Giao diện đồ họa sẽ tự mở)
ros2 launch mecanum_ga_pkg start_training.launch.py
```

*Lưu ý: Robot sẽ được spawn sau khoảng 8-10 giây để đảm bảo giao diện đồ họa đã sẵn sàng.*

---

## 2. Kiểm tra Dữ liệu Cảm biến

Mở các Terminal mới để chạy các lệnh kiểm tra sau:

### Xem Tọa độ và Fitness (Real-time)
Node này sẽ in ra vị trí (x, y) hiện tại và tổng quãng đường robot đã di chuyển.
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && . install/setup.bash
ros2 run mecanum_ga_pkg fitness_test --ros-args -p use_sim_time:=true
```

### Xem Dữ liệu Lidar (Dạng số)
Lọc để xem 20 giá trị khoảng cách đầu tiên từ cảm biến quét.
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /scan --use-sim-time | grep -A 20 "ranges:"
```

### Xem Lidar Trực quan (Rviz2)
```bash
ros2 run rviz2 rviz2
```
*Trong Rviz2:*
- **Fixed Frame**: Đổi thành `GA_8_TIA/lidar_link/lidar`
- **Add**: Chọn `LaserScan`
- **Topic**: Chọn `/scan`

---

## 3. Các Topic Quan trọng

| Topic | Loại dữ liệu | Mục đích |
|-------|--------------|----------|
| `/scan` | `sensor_msgs/LaserScan` | Dữ liệu quét từ Lidar 8 tia |
| `/model/GA_8_TIA/pose` | `geometry_msgs/Pose` | Vị trí và hướng của robot |
| `/model/GA_8_TIA/cmd_vel` | `geometry_msgs/Twist` | Gửi lệnh di chuyển cho robot |
| `/clock` | `rosgraph_msgs/Clock` | Thời gian từ Simulation (cần thiết cho `use_sim_time`) |

---

#---

## 5. Lệnh khởi chạy nhanh (Xem cả GUI, Fitness và Lidar)

Để bắt đầu làm việc nhanh nhất với đầy đủ giao diện và log dữ liệu, hãy copy và paste toàn bộ script sau vào Terminal:

```bash
# 1. Dọn dẹp & Khởi chạy hệ thống (Gazebo GUI + Bridge)
pkill -9 -f "gz sim|parameter_bridge|ruby" && sleep 2
cd ~/ros2_ws && colcon build --packages-select mecanum_ga_pkg --symlink-install
source install/setup.bash
ros2 launch mecanum_ga_pkg start_training.launch.py &

# 2. Đợi hệ thống ổn định rồi mở log Fitness và Lidar
sleep 15
# Mở log Fitness
ros2 run mecanum_ga_pkg fitness_test --ros-args -p use_sim_time:=true &
# Mở log Lidar (rút gọn)
ros2 topic echo /scan --use-sim-time --truncate-length 10 | grep -A 10 "ranges:"
```

