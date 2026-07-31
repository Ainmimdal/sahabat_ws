import math

import pytest

from shbat_pkg.junctek_kgf import (
    JunctekProtocolError,
    SETTING_SPECS,
    build_read_command,
    build_write_command,
    kgf_checksum,
    parse_measurement,
    parse_response,
    parse_settings,
)


def test_read_and_write_commands_match_manual_examples():
    assert build_read_command(50, 2) == b':R50=2,2,1,\r\n'
    assert build_write_command(20, 2000, 1) == b':W20=1,216,2000,\r\n'
    assert kgf_checksum([2000]) == 216


def test_parse_manual_measurement_example():
    line = (
        ':r50=2,215,2056,200,5408,4592,9437,14353,'
        '134,4112,0,0,162,30682,'
    )
    measurement = parse_measurement(line)
    assert measurement.address == 2
    assert measurement.voltage_v == pytest.approx(20.56)
    assert measurement.current_magnitude_a == pytest.approx(2.0)
    assert measurement.remaining_ah == pytest.approx(5.408)
    assert measurement.energy_wh == pytest.approx(94.37)
    assert measurement.temperature_c == pytest.approx(34.0)
    assert measurement.power_magnitude_w == pytest.approx(41.12)
    assert measurement.output_status_name == 'on'
    assert measurement.internal_resistance_ohm == pytest.approx(0.30682)


def test_parse_manual_settings_example():
    line = (
        ':r51=1,211,3000,100,2000,2000,10000,151,10,7,'
        '200,120,90,101,0,0,2,12,13,'
    )
    settings = parse_settings(line)
    assert settings.over_voltage_v == pytest.approx(30.0)
    assert settings.under_voltage_v == pytest.approx(1.0)
    assert settings.positive_over_current_a == pytest.approx(20.0)
    assert settings.negative_over_current_a == pytest.approx(20.0)
    assert settings.capacity_ah == pytest.approx(20.0)
    assert settings.voltage_calibration == 20
    assert settings.current_calibration == -10
    assert settings.temperature_calibration == 1


def test_setting_encoding_and_range_validation():
    assert SETTING_SPECS['capacity_ah'].encode(200.0) == 2000
    assert SETTING_SPECS['over_temperature_c'].encode(50.0) == 150
    with pytest.raises(ValueError):
        SETTING_SPECS['over_voltage_v'].encode(math.inf)
    with pytest.raises(ValueError):
        SETTING_SPECS['over_voltage_v'].encode(121.0)


def test_bad_checksum_is_rejected():
    with pytest.raises(JunctekProtocolError, match='checksum mismatch'):
        parse_response(
            ':r50=2,1,2056,200,5408,4592,9437,14353,'
            '134,4112,0,0,162,30682,',
            expected_function=50,
            validate_checksum=True,
        )
