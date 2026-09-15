import importlib
import unittest
from unittest import mock


utils_module = importlib.import_module('crtk.utils')


class FakeClock:
    def __init__(self, on_sleep=None):
        self.seconds = 100.0
        self.sleep_calls = 0
        self.on_sleep = on_sleep

    def monotonic(self):
        return self.seconds

    def time_ns(self):
        return int(self.seconds * 1e9)

    def sleep(self, duration):
        self.sleep_calls += 1
        self.seconds += duration
        if self.on_sleep:
            self.on_sleep(self)


class TestWaitForReceived(unittest.TestCase):
    def setUp(self):
        self.utils = object.__new__(utils_module.utils)
        self.utils._utils__connection_timeout = 0.1
        self.utils._utils__connection_timeout_ns = int(0.1 * 1e9)
        self.wait = self.utils._utils__wait_for_received

    def patch_clock(self, clock):
        return mock.patch.multiple(
            utils_module.time,
            monotonic=clock.monotonic,
            time_ns=clock.time_ns,
            sleep=clock.sleep,
        )

    def test_default_waits_for_first_message(self):
        last_received = [0]

        def receive_after_20_ms(clock):
            if clock.seconds >= 100.02:
                last_received[0] = clock.time_ns()

        clock = FakeClock(on_sleep=receive_after_20_ms)
        with self.patch_clock(clock):
            self.wait(lambda: last_received[0], None)

        self.assertGreaterEqual(clock.seconds, 100.02)
        self.assertLess(clock.seconds, 100.1)

    def test_default_raises_after_connection_timeout(self):
        clock = FakeClock()
        with self.patch_clock(clock):
            with self.assertRaisesRegex(TimeoutError, 'within 0.1 seconds'):
                self.wait(lambda: 0, None)

        self.assertGreaterEqual(clock.seconds, 100.1)

    def test_default_does_not_wait_for_stale_data(self):
        clock = FakeClock()
        with self.patch_clock(clock):
            self.wait(lambda: 1, None)

        self.assertEqual(clock.sleep_calls, 0)

    def test_zero_timeout_is_non_blocking(self):
        clock = FakeClock()
        with self.patch_clock(clock):
            with self.assertRaisesRegex(TimeoutError, 'within 0.0 seconds'):
                self.wait(lambda: 0, 0.0)

        self.assertEqual(clock.sleep_calls, 0)


if __name__ == '__main__':
    unittest.main()
