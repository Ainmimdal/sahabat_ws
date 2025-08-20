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
        ]),
        ('share/' + package_name + '/config', [
            'config/mapper_params_online_async.yaml',
            'config/nav2_params.yaml'
        ]),
 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sahabat',
    maintainer_email='sahabat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpm_to_odom = shbat_pkg.rpm_to_odom:main',
            'kalman_filter = shbat_pkg.kalman_filter:main',
            'joy2cmd = shbat_pkg.joy2cmd:main',
            'base_controller = shbat_pkg.base_controller:main',
        ],
    },
)