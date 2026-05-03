import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def generate_launch_description():
    package_name = 'mecanum_ga_pkg'
    pkg_share_dir = get_package_share_directory(package_name)
    world_path = os.path.join(pkg_share_dir, 'models', 'mecung.sdf')
    models_dir = os.path.join(pkg_share_dir, 'models')

    # Argument để chọn chế độ Manual hay Training
    manual_arg = DeclareLaunchArgument(
        'manual', default_value='false',
        description='Nếu là true, sẽ không chạy robot_controller và training_node'
    )
    is_manual = LaunchConfiguration('manual')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_dir]
    )

    # 0. Dọn dẹp mạnh tay các tiến trình cũ để tránh hiện tượng "chớp" do trùng tên thực thể
    kill_old_processes = ExecuteProcess(
        cmd=['pkill', '-9', '-f', 'gz sim|parameter_bridge|ruby|robot_controller|training_node'],
        output='screen'
    )

    # 1. Khởi động Gazebo Sim
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 2. Unpause Gazebo
    unpause = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/congminh/control', 
             '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean', 
             '--timeout', '2000', '--req', 'pause: false'],
        output='screen'
    )

    # 3. Spawn 5 Robots with Namespaces
    def create_spawn_robot(i):
        # Mỗi robot cách nhau 0.5m để tránh va chạm lúc đầu
        x_pos = (i - 2) * 0.5 
        return ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/congminh/create',
                '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000', 
                '--req', f'sdf_filename: "minh_lidar_GA.sdf", name: "robot_{i}", pose: {{position: {{x: {x_pos}, y: 1.1, z: 0.1}}, orientation: {{x: 0, y: 0, z: -0.7071, w: 0.7071}}}}'],
            output='screen'
        )

    spawn_robots = [create_spawn_robot(i) for i in range(5)]

    # 4. ROS GZ Bridge for 5 Robots
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    for i in range(5):
        bridge_args.extend([
            f'/world/congminh/model/robot_{i}/link/khung_xe/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/model/robot_{i}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
            f'/model/robot_{i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=bridge_args,
        parameters=[{'use_sim_time': True}], output='screen'
    )

    # 5. Robot Controllers (1 for each robot)
    robot_controllers = []
    for i in range(5):
        robot_controllers.append(Node(
            package=package_name, executable='robot_controller',
            namespace=f'robot_{i}',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/robot_genome', '/robot_genome'), # Global topic for all
                ('/scan', f'/world/congminh/model/robot_{i}/link/khung_xe/sensor/lidar/scan'),
                ('/cmd_vel', f'/model/robot_{i}/cmd_vel')
            ],
            output='screen',
            condition=UnlessCondition(is_manual)
        ))

    # 6. Training Node (Manages all 5)
    training_node = Node(
        package=package_name, executable='training_node',
        parameters=[{'use_sim_time': True}], output='screen',
        condition=UnlessCondition(is_manual)
    )

    ld = LaunchDescription([
        manual_arg,
        set_gz_resource_path,
        kill_old_processes,
        TimerAction(period=3.0, actions=[gz_sim]),
        TimerAction(period=8.0, actions=[unpause]),
        TimerAction(period=12.0, actions=[bridge]),
        training_node
    ])

    # Add robots and controllers to LD
    for i in range(5):
        ld.add_action(TimerAction(period=10.0, actions=[spawn_robots[i]]))
        ld.add_action(TimerAction(period=14.0, actions=[robot_controllers[i]]))

    return ld
