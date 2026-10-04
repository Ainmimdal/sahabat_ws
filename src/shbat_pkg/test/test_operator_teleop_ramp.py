"""Remote teleop ramps up and down, but safety stops stay immediate."""

import threading
from types import SimpleNamespace

from shbat_pkg.operator_backend import OperatorBackend


class _Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _backend():
    clock = [100.0]
    backend = object.__new__(OperatorBackend)
    backend._now = lambda: clock[0]
    backend.max_linear = 0.5
    backend.max_angular = 1.2
    backend.teleop_linear_accel = 0.6
    backend.teleop_angular_accel = 1.2
    backend.teleop_timeout = 0.25
    backend.remote_active = False
    backend.remote_releasing = False
    backend.remote_target = (0.0, 0.0)
    backend.remote_output = (0.0, 0.0)
    backend.last_remote_ramp = 0.0
    backend.last_remote_command = 0.0
    backend.remote_lock = threading.RLock()
    backend.remote_pub = _Recorder()
    backend.remote_active_pub = _Recorder()
    return backend, clock


def _advance(backend, clock, seconds, step=0.05):
    for _ in range(round(seconds / step)):
        clock[0] += step
        backend._step_remote_ramp()


def _last(backend):
    message = backend.remote_pub.messages[-1]
    return message.linear.x, message.angular.z


def test_remote_command_ramps_up_instead_of_jumping():
    backend, clock = _backend()
    backend._set_remote_target(0.5, 0.0)
    assert _last(backend) == (0.0, 0.0)

    _advance(backend, clock, 0.5)
    assert abs(_last(backend)[0] - 0.3) < 1e-9  # 0.6 m/s^2 for 0.5 s

    _advance(backend, clock, 0.5)
    assert _last(backend)[0] == 0.5


def test_release_ramps_down_and_keeps_control_until_stopped():
    backend, clock = _backend()
    backend._set_remote_target(0.5, 1.2)
    _advance(backend, clock, 1.5)
    assert _last(backend) == (0.5, 1.2)

    backend._release_remote()
    _advance(backend, clock, 0.4)
    linear, angular = _last(backend)
    assert 0.2 < linear < 0.5 and 0.0 < angular < 1.2
    # The arbiter must keep selecting remote while it ramps down.
    assert backend.remote_active is True
    assert backend.remote_active_pub.messages[-1].data is True

    _advance(backend, clock, 1.0)
    assert _last(backend) == (0.0, 0.0)
    assert backend.remote_active is False
    assert backend.remote_active_pub.messages[-1].data is False


def test_safety_stop_is_immediate_even_while_moving():
    backend, clock = _backend()
    backend._set_remote_target(0.5, 0.0)
    _advance(backend, clock, 1.0)

    backend._stop_remote()

    assert _last(backend) == (0.0, 0.0)
    assert backend.remote_active is False
    clock[0] += 0.05
    count = len(backend.remote_pub.messages)
    backend._step_remote_ramp()
    assert len(backend.remote_pub.messages) == count


def test_new_command_during_release_resumes_from_current_speed():
    backend, clock = _backend()
    backend._set_remote_target(0.5, 0.0)
    _advance(backend, clock, 1.0)
    backend._release_remote()
    _advance(backend, clock, 0.25)
    slowed = _last(backend)[0]

    backend._set_remote_target(0.5, 0.0)

    assert backend.remote_releasing is False
    assert abs(_last(backend)[0] - slowed) < 1e-9


def test_deadman_release_from_browser_ramps_instead_of_stopping():
    backend, clock = _backend()
    backend.estop_active = False
    backend.last_sequence = None
    backend._lease_valid = lambda _lease: True
    backend._set_remote_target(0.5, 0.0)
    _advance(backend, clock, 1.0)

    released = SimpleNamespace(
        lease_id='lease', sequence=1, deadman=False,
        twist=SimpleNamespace(linear=SimpleNamespace(x=0.0),
                              angular=SimpleNamespace(z=0.0)),
    )
    OperatorBackend._teleop(backend, released)

    assert backend.remote_releasing is True
    assert _last(backend)[0] > 0.4
