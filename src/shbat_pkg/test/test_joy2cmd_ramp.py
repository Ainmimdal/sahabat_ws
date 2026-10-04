"""Physical joystick keeps control while its command ramps to zero."""

from types import SimpleNamespace

from shbat_pkg.joy2cmd import Joy2CmdNode


class _Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Time:
    def __init__(self, t):
        self.t = t

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=int((self.t - other.t) * 1e9))


def _node():
    clock = [0.0]
    node = object.__new__(Joy2CmdNode)
    node.get_clock = lambda: SimpleNamespace(now=lambda: _Time(clock[0]))
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_: None, warn=lambda *_: None)
    node.max_linear_speed = 0.5
    node.max_angular_speed = 1.0
    node.deadzone = 0.12
    node.linear_accel_limit = 0.6
    node.angular_accel_limit = 1.2
    node.allow_estop_clear = True
    node.ESTOP_BUTTON = 0
    node.RESUME_BUTTON = 1
    node.prev_buttons = []
    node.emergency_stopped = False
    node.localization_recovery_active = False
    node.last_linear_cmd = 0.0
    node.last_angular_cmd = 0.0
    node.last_cmd_time = _Time(0.0)
    node.last_joy_time = _Time(0.0)
    node.joy_silence_timeout = 0.15
    node.publisher_ = _Recorder()
    node.active_pub = _Recorder()
    node.stop_status_pub = _Recorder()
    node.estop_pub = _Recorder()
    return node, clock


def _joy(linear):
    return SimpleNamespace(axes=[0.0, linear], buttons=[0, 0])


def _drive(node, clock, linear, seconds, step=0.05):
    for _ in range(round(seconds / step)):
        clock[0] += step
        node.joy_callback(_joy(linear))


def test_joystick_stays_active_until_ramp_reaches_zero():
    node, clock = _node()
    _drive(node, clock, 1.0, 1.0)
    assert abs(node.publisher_.messages[-1].linear.x - 0.5) < 1e-9

    _drive(node, clock, 0.0, 0.4)
    # Stick centred, still slowing down: the arbiter must keep the joystick.
    assert 0.0 < node.publisher_.messages[-1].linear.x < 0.5
    assert node.active_pub.messages[-1].data is True

    _drive(node, clock, 0.0, 1.0)
    assert node.publisher_.messages[-1].linear.x == 0.0
    assert node.active_pub.messages[-1].data is False


def test_ramp_continues_when_joy_messages_stop():
    node, clock = _node()
    _drive(node, clock, 1.0, 1.0)
    count = len(node.publisher_.messages)

    clock[0] += 0.1  # within the silence timeout: nothing yet
    node.coast_to_stop()
    assert len(node.publisher_.messages) == count

    for _ in range(40):
        clock[0] += 0.05
        node.coast_to_stop()
    assert node.publisher_.messages[-1].linear.x == 0.0
    assert node.active_pub.messages[-1].data is False


def test_estop_still_stops_immediately():
    node, clock = _node()
    _drive(node, clock, 1.0, 1.0)
    clock[0] += 0.05
    node.joy_callback(SimpleNamespace(axes=[0.0, 1.0], buttons=[1, 0]))
    assert node.publisher_.messages[-1].linear.x == 0.0
    assert node.estop_pub.messages[-1].data is True
    assert node.active_pub.messages[-1].data is False
