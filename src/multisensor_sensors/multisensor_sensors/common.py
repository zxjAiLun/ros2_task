import math
import random
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def create_qos_from_params(node: Node, prefix: str) -> QoSProfile:
    reliability_str = node.declare_parameter(f'{prefix}.reliability', 'best_effort').get_parameter_value().string_value
    history_str = node.declare_parameter(f'{prefix}.history', 'keep_last').get_parameter_value().string_value
    depth = node.declare_parameter(f'{prefix}.depth', 10).get_parameter_value().integer_value

    reliability = ReliabilityPolicy.BEST_EFFORT if reliability_str.lower() == 'best_effort' else ReliabilityPolicy.RELIABLE
    history = HistoryPolicy.KEEP_LAST if history_str.lower() == 'keep_last' else HistoryPolicy.KEEP_ALL

    return QoSProfile(
        reliability=reliability,
        history=history,
        depth=depth,
    )


def get_publish_rate_hz(node: Node, default: float = 10.0) -> float:
    return node.declare_parameter('publish_rate', default).get_parameter_value().double_value


def angle_deg_normalized(angle: float) -> float:
    value = math.fmod(angle, 360.0)
    if value <= -180.0:
        value += 360.0
    elif value > 180.0:
        value -= 360.0
    return value


def random_bool_with_probability(p_true: float) -> bool:
    return random.random() < p_true


def load_simulation_params(node: Node, defaults: Dict[str, Any]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, default in defaults.items():
        if isinstance(default, bool):
            param = node.declare_parameter(key, default).get_parameter_value().bool_value
        elif isinstance(default, int):
            param = node.declare_parameter(key, default).get_parameter_value().integer_value
        elif isinstance(default, float):
            param = node.declare_parameter(key, default).get_parameter_value().double_value
        elif isinstance(default, str):
            param = node.declare_parameter(key, default).get_parameter_value().string_value
        else:
            param = default
        result[key] = param
    return result

