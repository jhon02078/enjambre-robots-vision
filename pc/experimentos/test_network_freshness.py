import sys
import threading
import unittest
from pathlib import Path


EXPERIMENT_DIR = Path(__file__).resolve().parent
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

from snapshot_controller import DISCOVERY_PORT, MultiRobotApp


class NetworkFreshnessTests(unittest.TestCase):
    def test_regular_discovery_uses_only_known_unicast_endpoints(self):
        app = MultiRobotApp.__new__(MultiRobotApp)
        app.lock = threading.Lock()
        app.discovered = {
            2: {"ip": "192.0.2.2", "port": 44444, "t": 10.0},
            3: {"ip": "192.0.2.3", "port": 44444, "t": 10.0},
        }

        endpoints = app._discovery_endpoints(include_broadcast=False)

        self.assertEqual(
            endpoints,
            {
                ("192.0.2.2", DISCOVERY_PORT),
                ("192.0.2.3", DISCOVERY_PORT),
            },
        )
        self.assertNotIn(("255.255.255.255", DISCOVERY_PORT), endpoints)

    def test_ack_refreshes_cached_robot_timestamp(self):
        app = MultiRobotApp.__new__(MultiRobotApp)
        app.lock = threading.Lock()
        app.discovered = {
            2: {"ip": "192.0.2.2", "port": 44444, "t": 10.0},
            3: None,
        }

        self.assertTrue(app._mark_robot_network_rx(2, now=15.0))
        self.assertEqual(app.discovered[2]["t"], 15.0)
        self.assertEqual(app.discovered[2]["last_rx_t"], 15.0)

    def test_unknown_robot_ack_does_not_create_endpoint(self):
        app = MultiRobotApp.__new__(MultiRobotApp)
        app.lock = threading.Lock()
        app.discovered = {2: None}

        self.assertFalse(app._mark_robot_network_rx(2, now=15.0))
        self.assertIsNone(app.discovered[2])

    def test_ack_requests_are_rate_limited_per_robot(self):
        app = MultiRobotApp.__new__(MultiRobotApp)
        app.lock = threading.Lock()
        app.last_ack_probe_perf = {2: 0.0, 3: 0.0}
        app.command_ack_probe_interval_s = 0.75

        self.assertTrue(app._should_request_command_ack(2, 10.0))
        self.assertFalse(app._should_request_command_ack(2, 10.2))
        self.assertTrue(app._should_request_command_ack(3, 10.2))
        self.assertTrue(app._should_request_command_ack(2, 10.8))

    def test_ack_requests_can_be_disabled_for_navigation(self):
        app = MultiRobotApp.__new__(MultiRobotApp)
        app.lock = threading.Lock()
        app.last_ack_probe_perf = {2: 0.0}
        app.command_ack_probe_interval_s = 0.0

        self.assertFalse(app._should_request_command_ack(2, 10.0))


if __name__ == "__main__":
    unittest.main()
