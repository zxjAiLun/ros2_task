import math

import pytest

multisensor_interfaces = pytest.importorskip('multisensor_interfaces')

from multisensor_fusion.fusion_node import FusionNode
from multisensor_interfaces.msg import Vision, Audio, Lidar


def make_msgs(angle_v: float, angle_a: float, detected_v: bool = True, detected_a: bool = True):
    v = Vision()
    v.detected = detected_v
    v.confidence = 0.9
    v.angle = angle_v

    a = Audio()
    a.voice_detected = detected_a
    a.energy = 0.8
    a.direction = angle_a

    l = Lidar()
    l.obstacle_detected = False
    l.min_distance = 10.0
    return v, a, l


def test_fusion_both_close_angles():
    node = FusionNode()
    v, a, l = make_msgs(10.0, 15.0)
    status, conf, angle = node.compute_fusion(v, a, l)
    assert status == 'both'
    assert conf > 0.0
    assert not math.isnan(angle)


def test_fusion_conflict_angles():
    node = FusionNode()
    v, a, l = make_msgs(0.0, 180.0)
    status, conf, angle = node.compute_fusion(v, a, l)
    assert status == 'conflict'
    assert conf == 0.0
    assert angle is None

