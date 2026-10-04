"""Focused tests for motor quick-stop and route pass-through configuration."""

from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml

from shbat_pkg.base_controller import BaseController
from shbat_pkg.zlac8015d import ZLAC8015D


class _FakeModbusClient:
    def __init__(self, **_kwargs):
        self.calls = []

    def connect(self):
        return True

    def close(self):
        pass

    def write_register(self, **kwargs):
        self.calls.append(('write_register', kwargs))
        return object()

    def write_registers(self, **kwargs):
        self.calls.append(('write_registers', kwargs))
        return object()


class _FakeLogger:
    def __init__(self):
        self.errors = []
        self.infos = []
        self.warnings = []

    def error(self, message):
        self.errors.append(message)

    def info(self, message):
        self.infos.append(message)

    def warn(self, message):
        self.warnings.append(message)


def test_zlac_normal_and_quick_stop_use_separate_registers(monkeypatch):
    client = _FakeModbusClient()
    monkeypatch.setattr(
        ZLAC8015D,
        'ModbusClient',
        lambda **_kwargs: client,
    )
    driver = ZLAC8015D.Controller(port='/dev/not-opened-in-test')

    driver.set_decel_time(500, 500)
    driver.set_quick_stop_mode(6)
    driver.set_quick_stop_decel_time(10, 10)
    driver.emergency_stop()

    assert client.calls[-4:] == [
        (
            'write_registers',
            {'address': 0x2082, 'values': [500, 500], 'device_id': 1},
        ),
        (
            'write_register',
            {'address': 0x2011, 'value': 6, 'device_id': 1},
        ),
        (
            'write_registers',
            {'address': 0x2084, 'values': [10, 10], 'device_id': 1},
        ),
        (
            'write_register',
            {'address': 0x200E, 'value': 0x05, 'device_id': 1},
        ),
    ]

    with pytest.raises(ValueError, match='mode must be 5, 6, or 7'):
        driver.set_quick_stop_mode(4)


def test_failed_zero_target_does_not_skip_emergency_quick_stop():
    calls = []

    class Driver:
        def set_rpm(self, _left, _right):
            calls.append('zero_target')
            return None

        def emergency_stop(self):
            calls.append('quick_stop')
            return object()

        def disable_motor(self):
            calls.append('disable')
            return object()

    logger = _FakeLogger()
    controller = SimpleNamespace(
        driver=Driver(),
        target_linear_vel=0.4,
        target_angular_vel=0.2,
        get_logger=lambda: logger,
        _require_modbus_success=BaseController._require_modbus_success,
    )

    BaseController.stop_motors(controller)

    assert calls == ['zero_target', 'quick_stop']
    assert controller.target_linear_vel == 0.0
    assert controller.target_angular_vel == 0.0
    assert logger.errors


def test_clearing_estop_zeros_target_before_reenabling_driver():
    calls = []

    class Driver:
        def set_rpm(self, _left, _right):
            calls.append('zero_target')
            return object()

        def clear_alarm(self):
            calls.append('clear_quick_stop')
            return object()

        def enable_motor(self):
            calls.append('enable')
            return object()

    class Clock:
        def now(self):
            return 'now'

    logger = _FakeLogger()
    controller = SimpleNamespace(
        driver=Driver(),
        emergency_stopped=True,
        get_logger=lambda: logger,
        get_clock=lambda: Clock(),
        _require_modbus_success=BaseController._require_modbus_success,
    )

    BaseController.emergency_stop_callback(
        controller,
        SimpleNamespace(data=False),
    )

    assert calls == ['zero_target', 'clear_quick_stop', 'enable']
    assert controller.last_cmd_vel_time == 'now'
    assert controller.emergency_stopped is False


def test_route_behavior_uses_restored_follow_path_controller():
    package_root = Path(__file__).resolve().parents[1]
    parameters = yaml.safe_load(
        (package_root / 'config' / 'nav2_odom_only.yaml').read_text()
    )['controller_server']['ros__parameters']

    assert parameters['controller_plugins'] == ['FollowPath']

    behavior_tree = ET.parse(
        package_root
        / 'behavior_trees'
        / 'navigate_through_poses_smooth_replan.xml'
    )
    follow_path = behavior_tree.getroot().find('.//FollowPath')
    assert follow_path is not None
    assert follow_path.attrib['controller_id'] == 'FollowPath'


def test_nav2_uses_rotation_shim_over_dwb_with_lidar_safe_motion_limits():
    package_root = Path(__file__).resolve().parents[1]
    configuration = yaml.safe_load(
        (package_root / 'config' / 'nav2_odom_only.yaml').read_text()
    )
    controller_parameters = configuration['controller_server']['ros__parameters']
    smoother_parameters = configuration['velocity_smoother']['ros__parameters']
    planner = configuration['planner_server']['ros__parameters']['GridBased']

    controller = controller_parameters['FollowPath']
    progress_checker = controller_parameters['progress_checker']
    assert progress_checker['plugin'] == 'nav2_controller::PoseProgressChecker'
    assert progress_checker['required_movement_angle'] == 0.20
    assert controller['plugin'] == (
        'nav2_rotation_shim_controller::RotationShimController'
    )
    assert controller['primary_controller'] == 'dwb_core::DWBLocalPlanner'
    # Pivots must stay within the lidar-safe angular limit and ramp open loop.
    assert controller['rotate_to_heading_angular_vel'] <= controller['max_vel_theta']
    assert controller['closed_loop'] is False
    assert (
        controller['angular_disengage_threshold']
        < controller['angular_dist_threshold']
    )
    assert controller['max_vel_x'] == 0.4
    assert controller['max_vel_y'] == 0.0
    assert controller['max_vel_theta'] == 0.5
    assert controller['min_speed_theta'] == 0.15
    assert controller['acc_lim_theta'] == 3.0
    assert controller['decel_lim_theta'] == -3.0
    assert controller['vx_samples'] == 10
    assert controller['vtheta_samples'] == 20
    assert controller['sim_time'] == 1.2
    assert controller['publish_evaluation'] is False
    assert controller['trans_stopped_velocity'] == 0.05
    assert controller['critics'][:2] == ['RotateToGoal', 'Oscillation']
    assert controller['RotateToGoal.slowing_factor'] == 8.0

    assert planner['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert planner['tolerance'] == 0.25
    assert planner['downsample_costmap'] is False
    assert planner['downsampling_factor'] == 1
    assert planner['smooth_path'] is False
    assert smoother_parameters['max_velocity'] == [0.5, 0.0, 0.5]
    assert smoother_parameters['min_velocity'] == [-0.3, 0.0, -0.5]
    # Gentle build-up for the slow drive; prompt braking is unchanged.
    assert smoother_parameters['max_accel'] == [0.3, 0.0, 0.8]
    assert controller['acc_lim_x'] == smoother_parameters['max_accel'][0]
    # Scaling axes together would slave pivot braking to the linear ramp.
    assert smoother_parameters['scale_velocities'] is False
    assert smoother_parameters['feedback'] == 'OPEN_LOOP'
    assert smoother_parameters['max_decel'] == [-0.35, 0.0, -3.0]


def test_live_editor_launch_avoids_duplicate_api_and_namespaced_keepout_mask():
    package_root = Path(__file__).resolve().parents[1]
    localization_launch = (
        package_root / 'launch' / 'localization_patrol_launch.py'
    ).read_text()
    navigation_launch = (
        package_root / 'launch' / 'slam_nav_launch.py'
    ).read_text()
    rviz_configuration = yaml.safe_load(
        (package_root / 'rviz' / 'waypoint_editor.rviz').read_text()
    )

    assert "'use_api': 'false'" in localization_launch
    assert "'mask_topic': '/keepout_filter_mask'" in navigation_launch
    assert rviz_configuration['Visualization Manager']['Global Options'][
        'Frame Rate'
    ] == 15


def test_imu_frame_is_fixed_to_base_link_for_ekf_fusion():
    package_root = Path(__file__).resolve().parents[1]
    robot = ET.parse(
        package_root / 'urdf' / 'sahabat_robot.urdf.xacro'
    ).getroot()

    assert robot.find("./link[@name='imu_link']") is not None
    imu_joint = robot.find("./joint[@name='imu_joint']")
    assert imu_joint is not None
    assert imu_joint.attrib['type'] == 'fixed'
    assert imu_joint.find('parent').attrib['link'] == 'base_link'
    assert imu_joint.find('child').attrib['link'] == 'imu_link'
    assert imu_joint.find('origin').attrib['rpy'] == '0 0 0'

    navigation_launch = (
        package_root / 'launch' / 'slam_nav_launch.py'
    ).read_text()
    ekf_parameters = yaml.safe_load(
        (package_root / 'config' / 'ekf.yaml').read_text()
    )['ekf_filter_node']['ros__parameters']
    assert "'frame_id': 'imu_link'" in navigation_launch
    assert ekf_parameters['imu0'] == '/imu'


def test_ekf_takes_heading_from_gyro_not_wheel_yaw():
    package_root = Path(__file__).resolve().parents[1]
    ekf = yaml.safe_load(
        (package_root / 'config' / 'ekf.yaml').read_text()
    )['ekf_filter_node']['ros__parameters']
    # Index layout: x y z, roll pitch yaw, vx vy vz, vroll vpitch vyaw, ax ay az
    wheel, imu = ekf['odom0_config'], ekf['imu0_config']
    # Wheels over-count pivots by ~7.5%; no absolute wheel pose may be fused.
    assert wheel[:6] == [False] * 6
    assert wheel[6] is True
    # The gyro rate is the heading source; IMU orientation is not fused.
    assert imu[11] is True
    assert imu[:6] == [False] * 6
    # Without published covariance the gyro cannot outweigh wheel vyaw.
    for launch_file in ('slam_nav_launch.py', 'sahabat_launch.py',
                        'nav2_test_launch.py'):
        text = (package_root / 'launch' / launch_file).read_text()
        assert "'imu_angular_velocity_covariance'" in text, launch_file


def test_goal_pose_recovery_is_bounded_and_stationary():
    package_root = Path(__file__).resolve().parents[1]
    configuration = yaml.safe_load(
        (package_root / 'config' / 'nav2_odom_only.yaml').read_text()
    )
    behavior_tree = ET.parse(
        package_root
        / 'behavior_trees'
        / 'navigate_to_pose_replan_if_path_invalid.xml'
    )
    root = behavior_tree.getroot()

    assert configuration['bt_navigator']['ros__parameters'][
        'default_server_timeout'
    ] == 200
    path_timer = root.find('.//PathExpiringTimer')
    assert path_timer is not None
    assert path_timer.attrib['seconds'] == '20'
    assert root.find('.//GlobalUpdatedGoal') is not None
    assert root.find('.//IsPathValid') is not None
    recovery = root.find(".//RecoveryNode[@name='NavigateRecovery']")
    assert recovery is not None
    assert recovery.attrib['number_of_retries'] == '2'
    assert root.find('.//Spin') is None
    assert root.find('.//BackUp') is None
    assert root.find('.//Wait') is not None
    follow_path = root.find('.//FollowPath')
    assert follow_path is not None
    assert follow_path.attrib['controller_id'] == 'FollowPath'
