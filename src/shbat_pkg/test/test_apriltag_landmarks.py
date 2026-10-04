import numpy as np

from shbat_pkg.apriltag_landmark_manager import (
    AprilTagLandmarkManager,
    LandmarkRecord,
    angle_difference,
    derive_map_id,
    landmark_from_yaml,
    landmark_to_yaml,
    quaternion_to_matrix,
    robot_pose_from_tag,
    summarize_transforms,
    tag_map_path,
    yaw_from_matrix,
)


def _planar_transform(x=0.0, y=0.0, yaw=0.0):
    matrix = quaternion_to_matrix((
        0.0,
        0.0,
        np.sin(yaw / 2.0),
        np.cos(yaw / 2.0),
    ))
    matrix[:3, 3] = (x, y, 0.0)
    return matrix


def test_map_id_and_storage_paths_support_flat_and_directory_maps(tmp_path):
    assert derive_map_id('', '/maps/lobby.yaml') == 'lobby'
    assert derive_map_id('', '/maps/lobby/map.yaml') == 'lobby'
    assert derive_map_id('selected', '/maps/ignored.yaml') == 'selected'

    assert tag_map_path(tmp_path, 'flat') == tmp_path / 'flat_tags.yaml'
    directory_map = tmp_path / 'gallery'
    directory_map.mkdir()
    (directory_map / 'map.yaml').write_text('image: map.pgm\n', encoding='utf-8')
    assert tag_map_path(tmp_path, 'gallery') == directory_map / 'tags.yaml'


def test_robot_pose_is_recovered_from_saved_tag_and_camera_observation():
    map_to_base = _planar_transform(2.0, -1.0, 0.4)
    base_to_camera = _planar_transform(0.20, 0.0, 0.0)
    camera_to_tag = _planar_transform(1.8, 0.3, -0.2)
    map_to_tag = map_to_base @ base_to_camera @ camera_to_tag

    recovered = robot_pose_from_tag(
        map_to_tag,
        camera_to_tag,
        base_to_camera,
    )

    np.testing.assert_allclose(
        recovered[:3, 3], map_to_base[:3, 3], atol=1e-9
    )
    assert abs(angle_difference(
        yaw_from_matrix(recovered), yaw_from_matrix(map_to_base)
    )) < 1e-9


def test_capture_summary_rejects_one_large_translation_outlier():
    samples = [
        _planar_transform(1.00, 2.00, 0.20),
        _planar_transform(1.01, 1.99, 0.21),
        _planar_transform(0.99, 2.01, 0.19),
        _planar_transform(1.00, 2.01, 0.20),
        _planar_transform(8.00, -5.00, 2.50),
    ]

    average, count, translation_std, rotation_std = summarize_transforms(samples)

    assert count == 4
    np.testing.assert_allclose(
        average[:2, 3], (1.0, 2.0025), atol=0.01
    )
    assert translation_std < 0.02
    assert rotation_std < 0.02


def test_landmark_yaml_round_trip_keeps_pose_and_quality():
    record = LandmarkRecord(
        tag_id=12,
        name='lobby_north',
        family='36h11',
        size_m=0.20,
        matrix=_planar_transform(3.2, -0.4, 1.1),
        sample_count=18,
        translation_std_m=0.012,
        rotation_std_rad=0.021,
    )

    encoded = landmark_to_yaml(record)
    decoded = landmark_from_yaml(encoded, '36h11', 0.20)

    assert decoded.tag_id == record.tag_id
    assert decoded.name == record.name
    assert decoded.sample_count == record.sample_count
    np.testing.assert_allclose(decoded.matrix, record.matrix, atol=1e-6)


def test_trusted_map_pose_is_native_bool_for_ros_message_assignment():
    manager = object.__new__(AprilTagLandmarkManager)
    manager._map_pose_available = lambda: True
    manager._now = lambda: 10.0
    manager.last_amcl_pose_at = 9.5
    manager.amcl_covariance_good = np.bool_(True)

    result = manager._trusted_map_pose_available()

    assert type(result) is bool
    assert result is True


def test_family_names_match_with_or_without_tag_prefix():
    from shbat_pkg.apriltag_landmark_manager import normalize_family

    assert normalize_family('tag36h11') == normalize_family('36h11') == '36h11'
    assert normalize_family('tagStandard41h12') == 'Standard41h12'
