"""The swarm formation's geometry."""

import math

import numpy as np

from swarm.formation import formation_slots, rotate_body_to_ned


def test_rotation_heading_east_maps_forward_to_east() -> None:
    ned = rotate_body_to_ned(np.array([1.0, 0.0, 0.0]), math.pi / 2)
    assert np.allclose(ned, [0.0, 1.0, 0.0], atol=1e-9)


def test_formation_is_a_north_up_square_stepping_up_1_5_m_from_the_leader() -> None:
    slots = formation_slots(4)
    assert np.allclose(slots[0], [5, 0, -1.5], atol=1e-9)  # Alpha, north, lowest follower
    assert np.allclose(slots[1], [0, 5, -3], atol=1e-9)  # Bravo, east
    assert np.allclose(slots[2], [-5, 0, -4.5], atol=1e-9)  # Charlie, south
    assert np.allclose(slots[3], [0, -5, -6], atol=1e-9)  # Delta, west, highest


def test_three_followers_ring_120_degrees_apart_all_above() -> None:
    slots = formation_slots(3)
    assert [float(s[2]) for s in slots] == [-1.5, -3.0, -4.5]
    assert np.allclose(slots[1][:2], [5 * math.cos(2 * math.pi / 3), 5 * math.sin(2 * math.pi / 3)])


def test_leaders_fpv_camera_has_a_follower_in_frame_whichever_way_it_faces() -> None:
    slots = formation_slots(4)
    for heading_deg in range(0, 360, 5):
        in_frame = 0
        for north, east, _ in slots:
            bearing = math.degrees(math.atan2(east, north))
            off = (bearing - heading_deg + 180) % 360 - 180
            in_frame += abs(off) <= 60  # 120 degree FOV
        assert 1 <= in_frame <= 2
