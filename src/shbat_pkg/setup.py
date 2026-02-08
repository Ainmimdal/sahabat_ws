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
            'launch/sahabat_manual.launch.py',
            'launch/nav2_test_launch.py',
            'launch/slam_nav_launch.py',
            'launch/localization_patrol_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/mapper_params_online_async.yaml',
            'config/localization__params_online_async.yaml',
            'config/nav2_params.yaml',
            'config/nav2_params_odom.yaml',
            'config/nav2_odom_only.yaml',
            'config/zed_pos_tracking_override.yaml',
            'config/ekf.yaml',
            'config/amcl.yaml',
            'config/scan_filter.yaml',
            'config/patrol_waypoints.yaml',
            'config/slam_toolbox.yaml',
            'config/slam_toolbox_localization.yaml',
            'config/zed_depth_to_scan.yaml',
            'config/exhibit_routes.yaml',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/nav2_test.rviz',
            'rviz/slam_nav.rviz',
        ]),
 
    ],
    install_requires=[
        'setuptools',
        'pymodbus>=3.0.0',
        'numpy',
        'flask',
        'flask-cors',
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
            'scan_filter = shbat_pkg.scan_filter:main',
            'waypoint_patrol = shbat_pkg.waypoint_patrol:main',
            'waypoint_collector = shbat_pkg.waypoint_collector:main',
            'waypoint_panel = shbat_pkg.waypoint_panel:main',
            'waypoint_manager = shbat_pkg.waypoint_manager:main',
            'api_bridge = shbat_pkg.api_bridge:main',
            'save_current_pose = shbat_pkg.save_current_pose:main',
            'pose_saver_auto = shbat_pkg.pose_saver_auto:main',
            'exhibit_navigator = shbat_pkg.exhibit_navigator:main',
        ],
    },
)