import pytest

from rclpy.time import Time

multisensor_interfaces = pytest.importorskip('multisensor_interfaces')

from multisensor_monitor.status_monitor_node import StatusMonitorNode


def test_status_timeout_and_alert():
    node = StatusMonitorNode()
    node.timeout_sec = 1.0
    now = node.get_clock().now()
    node.last_update['vision_node'] = now - Time(seconds=2)

    node.maybe_publish_alert('vision_node', now, 2.0)
    assert node.last_alert_time['vision_node'] is not None

