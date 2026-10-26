from pathlib import Path

import pytest

from swarm.settings import SETTINGS_PATH, SettingsError, load_settings


def test_the_mission_file_loads() -> None:
    settings = load_settings()
    assert settings.copters >= 1
    assert settings.field.point_a != settings.field.point_b
    assert settings.sim.router in ("sitl", "field")
    assert set(settings.sim.sitl_params()) == {"SIM_WIND_SPD", "SIM_WIND_DIR", "SIM_WIND_TURB"}


SAMPLE = """
[mission]
point_a = [43.6304, -79.4010]
point_b = [43.6316, -79.4045]
copters = 5

[swarm]
follower_distance_m = 8.0
follower_height_step_m = 1.5
liftoff_max_leader_distance_m = 100.0
follow_max_distance_m = 150.0
liftoff_min_leader_height_m = 5.0
landing_zone_radius_m = 25.0

[fence]
enabled = true
corners = [[43.627, -79.407], [43.634, -79.407], [43.634, -79.393], [43.627, -79.393]]
max_height_m = 100.0
action = "brake"

[sim]
home = [43.6290, -79.3962]
ground_alt_m = "auto"
spawn_spacing_m = 5.0
router = "sitl"
wind_speed_ms = 0.0
wind_from_deg = 180.0
wind_turbulence_ms = 0.0
vtx_mw = 25.0
"""
"""A settings file to edit: the tests do not depend on what mission.toml holds today."""


def test_the_sample_has_every_key_the_mission_file_has() -> None:
    def keys(text: str) -> set[str]:
        lines = (line.lstrip("# ") for line in text.splitlines())  # commented out: still a key
        return {line.split("=")[0].strip() for line in lines if "=" in line}

    assert keys(SAMPLE) == keys(SETTINGS_PATH.read_text())


def _edited(tmp_path: Path, *edits: tuple[str, str]) -> Path:
    text = SAMPLE
    for old, new in edits:
        assert old in text
        text = text.replace(old, new, 1)
    path = tmp_path / "mission.toml"
    path.write_text(text)
    return path


def test_points_and_home_come_from_the_file(tmp_path: Path) -> None:
    path = _edited(
        tmp_path,
        ("point_b = [", "point_b = [44.0, -80.0]\n# was ["),
        ("home = [", "home = [44.1, -80.1]\n# was ["),
        ("enabled = true", "enabled = false"),  # both now far outside the fence
    )
    settings = load_settings(path)
    assert settings.field.point_b == (44.0, -80.0)
    home = settings.field.home
    assert (home.origin_lat_deg, home.origin_lon_deg) == (44.1, -80.1)


@pytest.mark.parametrize(
    ("old", "new", "error"),
    [
        ("follower_distance_m =", "radius =", "missing follower_distance_m"),
        (
            "follower_height_step_m = 1.5",
            "follower_height_step_m = 0",
            "follower_height_step_m must be more than 0",
        ),
        (
            "follow_max_distance_m = 150.0",
            "follow_max_distance_m = 80.0",
            "follow_max_distance_m must be more than",
        ),
        ("[swarm]", "[formation]", "no \\[swarm\\] table"),
        ("copters = 5", "copters = 0", "copters must be"),
        ('router = "sitl"', 'router = "wifi"', "router is 'wifi'"),
        ("point_a = [", "point_a = [1.0]\n# [", "point_a must be"),
        ('ground_alt_m = "auto"', 'ground_alt_m = "high"', "ground_alt_m must be a number"),
        ("vtx_mw = 25.0", "vtx_mw = 0", "vtx_mw must be more than 0"),
    ],
)
def test_mistakes_are_reported(tmp_path: Path, old: str, new: str, error: str) -> None:
    with pytest.raises(SettingsError, match=error):
        load_settings(_edited(tmp_path, (old, new)))


def test_a_missing_file_is_reported(tmp_path: Path) -> None:
    with pytest.raises(SettingsError, match="nope.toml"):
        load_settings(tmp_path / "nope.toml")


def test_ground_elevation_is_auto_or_a_number(tmp_path: Path) -> None:
    assert load_settings(_edited(tmp_path)).field.home_alt_m is None
    path = _edited(tmp_path, ('ground_alt_m = "auto"', "ground_alt_m = 75"))
    assert load_settings(path).field.home_alt_m == 75.0


def test_swarm_tuning_comes_from_the_file(tmp_path: Path) -> None:
    path = _edited(
        tmp_path,
        ("liftoff_max_leader_distance_m = 100.0", "liftoff_max_leader_distance_m = 60"),
        ("liftoff_min_leader_height_m = 5.0", "liftoff_min_leader_height_m = 8"),
        ("landing_zone_radius_m = 25.0", "landing_zone_radius_m = 15"),
    )
    swarm = load_settings(path).swarm
    assert (
        swarm.liftoff_max_leader_distance_m,
        swarm.liftoff_min_leader_height_m,
        swarm.landing_zone_radius_m,
    ) == (60, 8, 15)
    assert (
        swarm.follower_distance_m,
        swarm.follower_height_step_m,
        swarm.follow_max_distance_m,
    ) == (8.0, 1.5, 150.0)


def test_the_fence_comes_from_the_file(tmp_path: Path) -> None:
    fence = load_settings(_edited(tmp_path)).fence
    assert fence.enabled and fence.action == "brake" and fence.max_height_m == 100.0
    assert fence.corners[0] == (43.627, -79.407) and len(fence.corners) == 4


@pytest.mark.parametrize(
    ("old", "new", "error"),
    [
        ("point_b = [", "point_b = [43.64, -79.40]\n# was [", "point_b .* is outside the fence"),
        ("home = [", "home = [43.6290, -79.39303]\n# was [", "start of copter 2 .* outside"),
        (
            "corners = [[43.627, -79.407], [43.634, -79.407], ",
            "corners = [",
            "3 \\[latitude, longitude\\] or more",
        ),
        ("[43.634, -79.407], [43.634, -79.393]", "[43.634, -79.393], [43.634, -79.407]", "cross"),
        ('action = "brake"', 'action = "hover"', "action is 'hover'"),
        ("max_height_m = 100.0", "max_height_m = 5", "max_height_m must be 10 to 1000"),
        ("enabled = true", "enabled = 1", "enabled must be true or false"),
    ],
)
def test_fence_mistakes_are_reported(tmp_path: Path, old: str, new: str, error: str) -> None:
    with pytest.raises(SettingsError, match=error):
        load_settings(_edited(tmp_path, (old, new)))


def test_a_fence_without_corners_is_the_ceiling_alone(tmp_path: Path) -> None:
    path = _edited(
        tmp_path,
        ("corners = [", "# corners = ["),
        ("point_b = [", "point_b = [43.64, -79.40]\n# was ["),
    )
    fence = load_settings(path).fence
    assert fence.enabled and fence.corners == ()
    assert fence.params()["FENCE_TYPE"] == 1.0


def test_a_fence_turned_off_does_not_need_the_places_inside(tmp_path: Path) -> None:
    path = _edited(
        tmp_path,
        ("enabled = true", "enabled = false"),
        ("point_b = [", "point_b = [43.64, -79.40]\n# was ["),
    )
    assert not load_settings(path).fence.enabled
