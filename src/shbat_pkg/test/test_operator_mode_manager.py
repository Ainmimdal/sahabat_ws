"""Non-motion tests for the robot app's mode commands (nothing is launched)."""

from shbat_pkg.operator_mode_manager import full_stack_command


def test_full_mapping_matches_new_mapping_shortcut(tmp_path):
    """Mapping runs the canonical mapping stack without robot-side GUIs."""
    command = full_stack_command('mapping', '', tmp_path)
    assert command[:4] == ['ros2', 'launch', 'shbat_pkg', 'navigation.launch.py']
    assert 'mode:=mapping' in command
    assert 'use_rviz:=false' in command
    assert 'use_mapping_panel:=false' in command


def test_full_operations_matches_live_shortcut(tmp_path):
    """Operations mirrors live_waypoint_editor: same map, sets, ZED and API."""
    (tmp_path / 'cuteroom.yaml').write_text('image: cuteroom.pgm\n')
    command = full_stack_command('operations', 'cuteroom', tmp_path)
    assert command[:4] == ['ros2', 'launch', 'shbat_pkg', 'operations.launch.py']
    for argument in (
        f'map_file:={tmp_path / "cuteroom"}',
        f'maps_directory:={tmp_path}',
        'map_id:=cuteroom',
        'use_rviz:=false',
        'use_waypoint_gui:=false',
        'use_api:=true',
        'use_zed:=true',
        'localization_backend:=amcl',
    ):
        assert argument in command
    # The full profile starts the hardware drivers itself.
    assert not any(item.startswith('use_hardware') for item in command)


def test_full_operations_accepts_folder_maps(tmp_path):
    """Older maps saved as <maps>/<id>/map.yaml are still operable."""
    (tmp_path / 'lobby').mkdir()
    (tmp_path / 'lobby' / 'map.yaml').write_text('image: map.pgm\n')
    assert full_stack_command('operations', 'lobby', tmp_path) is not None


def test_full_operations_rejects_missing_or_unsafe_maps(tmp_path):
    """Unknown maps and path tricks never produce a command."""
    assert full_stack_command('operations', 'nope', tmp_path) is None
    assert full_stack_command('operations', '../etc', tmp_path) is None
    assert full_stack_command('operations', '', tmp_path) is None
    assert full_stack_command('localization', 'nope', tmp_path) is None
    assert full_stack_command('dance', '', tmp_path) is None
