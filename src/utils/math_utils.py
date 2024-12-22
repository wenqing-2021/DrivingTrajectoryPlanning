import numpy as np
import math


def norm_angle(angle) -> np.ndarray:
    """
    Normalize the angle to [-pi, pi]
    """
    if not isinstance(angle, np.ndarray):
        angle = np.array(angle)
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle
