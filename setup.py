from setuptools import find_packages, setup

package_name = 'mecanum_ga_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start_training.launch.py']),
        ('share/' + package_name + '/models', ['mecung.sdf', 'minh_lidar_GA.sdf']),
        ('share/' + package_name + '/config', ['config/fitness_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='congminh',
    maintainer_email='congminh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_test_node = mecanum_ga_pkg.lidar_test_node:main',
            'fitness_test = mecanum_ga_pkg.fitness_test:main',
            'network_test = mecanum_ga_pkg.network:main',
            'robot_controller = mecanum_ga_pkg.robot_controller:main',
            'training_node = mecanum_ga_pkg.training_node:main',
            'test_reset = mecanum_ga_pkg.test_reset:main',
        ],
    },
)
