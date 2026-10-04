from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_all_nav2_configs_use_only_lidar_observation_sources():
    config_names = (
        'nav2_odom_only.yaml',
        'nav2_params.yaml',
        'nav2_params_mapping.yaml',
        'nav2_params_odom.yaml',
    )

    for config_name in config_names:
        config_path = PACKAGE_ROOT / 'config' / config_name
        text = config_path.read_text(encoding='utf-8')
        data = yaml.safe_load(text)

        assert '/zed/' not in text
        assert 'PointCloud2' not in text

        observation_sources = []

        def collect(value):
            if isinstance(value, dict):
                for key, child in value.items():
                    if key == 'observation_sources':
                        observation_sources.append(child)
                    collect(child)
            elif isinstance(value, list):
                for child in value:
                    collect(child)

        collect(data)
        assert observation_sources
        assert all(source == 'scan' for source in observation_sources)


def test_zed_wrapper_is_image_only_and_apriltag_gated():
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'slam_nav_launch.py'
    ).read_text(encoding='utf-8')

    assert "'depth.depth_mode': 'NONE'" in launch_text
    assert 'depth.point_cloud_freq' not in launch_text
    assert 'point_cloud/cloud_registered' not in launch_text
    assert 'hardware_and(use_zed)' not in launch_text
    assert launch_text.count('condition=zed_apriltag_hardware_condition()') == 4

    operations_text = (
        PACKAGE_ROOT / 'launch' / 'operations.launch.py'
    ).read_text(encoding='utf-8')
    assert "('use_zed', 'true')" in operations_text
    assert "('use_apriltag', 'true')" in operations_text
