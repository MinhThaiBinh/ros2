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

    # 0. Dọn dẹp mạnh tay các tiến trình cũ
    # (Bỏ pkill trong LD vì nó gây xung đột SIGKILL với các node vừa khởi tạo)

    # 1. Khởi động Gazebo Sim (Bật GUI bằng cách bỏ -s)
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

    # 3. Spawn 1 Robot (Thay đổi từ 5 xuống 1 để đồng bộ với training_node)
    def create_spawn_robot():
        return ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/congminh/create',
                '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000', 
                '--req', 'sdf_filename: "minh_lidar_GA.sdf", name: "GA_8_TIA", pose: {position: {x: 0, y: 1.1, z: 0.1}, orientation: {x: 0, y: 0, z: -0.7071, w: 0.7071}}'],
            output='screen'
        )

    spawn_robot = create_spawn_robot()

    # 4. ROS GZ Bridge for 1 Robot
    # Bridge kết nối dữ liệu Gazebo sang ROS và ngược lại
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/world/congminh/model/GA_8_TIA/link/khung_xe/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/model/GA_8_TIA/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
        '/model/GA_8_TIA/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
    ]

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=bridge_args,
        parameters=[{'use_sim_time': True}], 
        remappings=[
            ('/world/congminh/model/GA_8_TIA/link/khung_xe/sensor/lidar/scan', '/scan'),
            ('/model/GA_8_TIA/cmd_vel', '/cmd_vel')
        ],
        output='screen'
    )

    # 5. Robot Controller
    robot_controller = Node(
        package=package_name, executable='robot_controller',
        # Bo namespace de dung topic tuyet doi cho de
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=UnlessCondition(is_manual)
    )

    # 6. Training Node
    training_node = Node(
        package=package_name, executable='training_node',
        parameters=[{'use_sim_time': True}], output='screen',
        condition=UnlessCondition(is_manual)
    )

    ld = LaunchDescription([
        manual_arg,
        set_gz_resource_path,
        TimerAction(period=2.0, actions=[gz_sim]),
        TimerAction(period=8.0, actions=[unpause]),
        TimerAction(period=12.0, actions=[bridge]),
        training_node,
        TimerAction(period=10.0, actions=[spawn_robot]),
        TimerAction(period=14.0, actions=[robot_controller])
    ])

    return ld
