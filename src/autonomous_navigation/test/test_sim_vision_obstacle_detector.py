import pathlib
import sys


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from autonomous_navigation.sim_vision_obstacle_core import (  # noqa: E402
    expand_box_cluster,
    global_to_local,
    visible_in_fov,
)


def test_global_to_local_forward_and_left():
    forward, left = global_to_local(
        robot_north=0.0,
        robot_west=0.0,
        robot_yaw=0.0,
        target_north=2.0,
        target_west=1.0,
    )
    assert forward == 2.0
    assert left == 1.0


def test_visible_in_fov_rejects_obstacle_outside_frustum():
    assert visible_in_fov(
        local_forward=2.0,
        local_left=3.0,
        fov_deg=70.0,
        min_distance_m=0.6,
        max_distance_m=6.0,
    ) is False


def test_expand_box_cluster_populates_center_cell():
    cluster = expand_box_cluster(
        center_north=2.5,
        center_west=0.0,
        size_north=0.8,
        size_west=0.8,
        cell_size=0.25,
        cost_value=25.0,
    )
    triples = [tuple(cluster[i:i + 3]) for i in range(0, len(cluster), 3)]
    assert (2.5, 0.0, 25.0) in triples
    assert len(triples) > 1
