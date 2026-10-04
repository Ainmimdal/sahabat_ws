"""Non-motion tests for operator localization readiness policy."""

from shbat_pkg.operator_backend import OperatorBackend


class TransformBufferHarness:
    """Return a configured result for the single TF readiness query."""

    def __init__(self, available=True):
        self.available = available

    def can_transform(self, *_args, **_kwargs):
        return self.available


def backend_harness(
    *,
    backend='amcl',
    pose_received=True,
    covariance_good=False,
):
    """Construct only the state used by the localization pose gate."""
    operator = object.__new__(OperatorBackend)
    operator.localization_backend = backend
    operator.last_amcl_pose = 1.0 if pose_received else 0.0
    operator.localization_covariance_good = covariance_good
    return operator


def health_harness(*, map_received=True, scan_age=0.0, tf_available=True):
    """Construct only the state used by map, scan, and TF health checks."""
    operator = object.__new__(OperatorBackend)
    operator.last_map = 1.0 if map_received else 0.0
    operator.last_scan = 10.0 - scan_age
    operator.tf_buffer = TransformBufferHarness(tf_available)
    operator._now = lambda: 10.0
    return operator


def test_static_map_remains_healthy_without_periodic_republication():
    """A received transient-local map must not expire like sensor data."""
    operator = health_harness(map_received=True)

    map_healthy, scan_healthy, tf_healthy = operator._health()

    assert map_healthy
    assert scan_healthy
    assert tf_healthy


def test_map_health_requires_at_least_one_map_message():
    """Do not report map readiness before receiving the latched map."""
    operator = health_harness(map_received=False)

    map_healthy, _scan_healthy, _tf_healthy = operator._health()

    assert not map_healthy


def test_amcl_production_gate_requires_converged_covariance():
    """Keep the strict covariance requirement as the production default."""
    operator = backend_harness(covariance_good=False)

    assert not operator._localization_pose_healthy('map')


def test_slam_toolbox_readiness_remains_frame_based():
    """Do not change the separate SLAM Toolbox readiness contract."""
    operator = backend_harness(backend='slam_toolbox')

    assert operator._localization_pose_healthy('map')
    assert not operator._localization_pose_healthy('odom')
