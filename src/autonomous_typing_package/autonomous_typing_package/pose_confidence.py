"""Shared X/Y stability limits; covariance is expressed in square metres."""
import math

MAX_XY_STDDEV_M = 0.020
MAX_XY_VARIANCE_M2 = MAX_XY_STDDEV_M ** 2


def xy_variance_is_acceptable(var_x, var_y, threshold=MAX_XY_VARIANCE_M2):
    return all(math.isfinite(value) and 0.0 <= value <= threshold
               for value in (var_x, var_y))
