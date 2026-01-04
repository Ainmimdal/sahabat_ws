from setuptools import find_packages, setup

package_name = 'shbat_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', [
            'urdf/sahabat_robot.urdf.xacro',
            'urdf/sahabat_robot_with_zed.urdf.xacro',
        ]),

        ('share/' + package_name + '/map', [
            'map/map_save.yaml',
            'map/map_save.pgm',
            'map/map_serial.data',
            'map/map_serial.posegraph'
        ]),
        
        ('share/' + package_name + '/launch', [
            'launch/sahabat_launch.py',
            'launch/sahabat_mapping.launch.py',
            'launch/sahabat_nav.launch.py',
            'launch/sahabat_manual.launch.py',
            'launch/sahabat_rtabmap.launch.py',
            'launch/sahabat_rtabmap_debug.launch.py',
            'launch/sahabat_sensor_fusion_only.launch.py',
            'launch/test_nav_odom_only.launch.py',
            'launch/zed_odom_test.launch.py',
            'launch/sahabat_rtabmap_zed_only.launch.py',
            'launch/sahabat_zed_record.launch.py',
            'launch/sahabat_rtabmap_from_bag.launch.py',
            'launch/sahabat_navigation_test.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/mapper_params_online_async.yaml',
            'config/localization__params_online_async.yaml',
            'config/nav2_params.yaml',
            'config/nav2_params_odom.yaml',
            'config/rtabmap_params.yaml',
            'config/zed_pos_tracking_override.yaml',
            'config/ekf.yaml',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/rtabmap.rviz',
            'rviz/zed_odom_test.rviz',
            'rviz/zed_rtabmap.rviz',
            'rviz/navigation_test.rviz',
        ]),
 
    ],
    install_requires=[
        'setuptools',
        'pymodbus>=3.0.0',
        'numpy',
    ],
    zip_safe=True,
    maintainer='sahabat',
    maintainer_email='sahabat@todo.todo',
    description='Sahabat robot ROS2 package with ZLAC8015D motor control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpm_to_odom = shbat_pkg.rpm_to_odom:main',
            'kalman_filter = shbat_pkg.kalman_filter:main',
            'joy2cmd = shbat_pkg.joy2cmd:main',
            'base_controller = shbat_pkg.base_controller:main',
            'test_motor = shbat_pkg.test_motor:main',
        ],
    },
)