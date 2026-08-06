"""Non-motion tests for stale Nav2 callback protection."""

from types import SimpleNamespace

from shbat_pkg.operator_backend import OperatorBackend


class FakeFuture:
    """Minimal future that returns one fixed value."""

    def __init__(self, value):
        """Store the value returned by result()."""
        self.value = value

    def result(self):
        """Return the configured value."""
        return self.value


class FakeGoalHandle:
    """Record cancellation and result callback registration."""

    accepted = True

    def __init__(self):
        """Initialize observable callback state."""
        self.cancelled = False
        self.result_callback = None

    def cancel_goal_async(self):
        """Record that a stale accepted goal was canceled."""
        self.cancelled = True

    def get_result_async(self):
        """Return an object that captures the registered result callback."""
        handle = self

        class ResultFuture:
            """Capture the callback without scheduling ROS work."""

            def add_done_callback(self, callback):
                """Save the callback for explicit test invocation."""
                handle.result_callback = callback

        return ResultFuture()


def backend_harness(generation=1):
    """Construct only the fields used by the goal callback methods."""
    backend = object.__new__(OperatorBackend)
    backend.goal_generation = generation
    backend.navigation_state = 'goal_pending'
    backend.cancel_requested = False
    backend.current_goal_handle = None
    return backend


def test_stale_goal_acceptance_is_cancelled():
    """Cancel a goal accepted after Pause or a replacement command."""
    backend = backend_harness(generation=2)
    handle = FakeGoalHandle()

    OperatorBackend._goal_started(backend, FakeFuture(handle), generation=1)

    assert handle.cancelled
    assert backend.current_goal_handle is None
    assert backend.navigation_state == 'goal_pending'


def test_stale_goal_result_cannot_overwrite_replacement_state():
    """Ignore the late result from a canceled goal after Resume starts."""
    backend = backend_harness(generation=1)
    handle = FakeGoalHandle()
    OperatorBackend._goal_started(backend, FakeFuture(handle), generation=1)
    assert backend.navigation_state == 'navigating'

    backend.goal_generation = 2
    backend.navigation_state = 'goal_pending'
    backend.current_goal_handle = 'replacement-goal'
    stale_result = FakeFuture(SimpleNamespace(status=5))
    handle.result_callback(stale_result)

    assert backend.navigation_state == 'goal_pending'
    assert backend.current_goal_handle == 'replacement-goal'
