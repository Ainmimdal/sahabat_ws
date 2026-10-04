import math

from shbat_pkg.dock_pose_initializer import DockPoseInitializer, amcl_covariance_good


def test_amcl_covariance_gate():
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = 0.05
    covariance[35] = 0.02
    assert amcl_covariance_good(covariance)
    covariance[35] = math.nan
    assert not amcl_covariance_good(covariance)
    covariance[35] = 0.02
    covariance[0] = 1.5
    assert not amcl_covariance_good(covariance)


def test_dock_file_is_read_per_map(tmp_path):
    path = tmp_path / 'dock.yaml'
    path.write_text('version: 1\ndock: {x: 1.5, y: -2.0, yaw: 0.3}\n', encoding='utf-8')
    assert DockPoseInitializer.read_dock_file(path) == (1.5, -2.0, 0.3)
    path.write_text('dock: {x: bad}\n', encoding='utf-8')
    assert DockPoseInitializer.read_dock_file(path) is None
    assert DockPoseInitializer.read_dock_file(tmp_path / 'missing.yaml') is None
