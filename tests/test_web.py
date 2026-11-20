"""The web version (DESIGN.md §18): its data as the page takes it, what is generated and checked
in for it, and the plant's generated C held to the JAX backend the game's own physics runs."""

import ctypes
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest
from skyflow_dynamics.backends import jax as sfd_jax

from drone_duels import webdata
from drone_duels.cli import same_generated
from drone_duels.content.store import load
from drone_duels.physics.quad.plant import Plant, airframe
from drone_duels.server.protocol import ContentMsg

CC = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")


def test_what_is_generated_is_current() -> None:
    for path, text in webdata.generated().items():
        assert path.exists(), f"{path}: missing: run `uv run duels gen-web`"
        assert same_generated(path.read_text(), text), f"{path}: stale: run `uv run duels gen-web`"


def test_generated_files_compare_by_value_not_by_the_last_digit() -> None:
    assert same_generated('{"a": [1.0, 2.5], "b": "x"}', '{"a": [1.0000000000001, 2.5], "b": "x"}')
    assert not same_generated('{"a": [1.0, 2.5]}', '{"a": [1.001, 2.5]}')
    assert not same_generated('{"a": [1.0]}', '{"a": [1.0, 2.0]}')
    assert not same_generated('{"a": true}', '{"a": 1}')
    assert not same_generated("void f(void);", "void g(void);")


def test_the_content_is_what_flies_in_the_page() -> None:
    content = load()
    msg = webdata.web_content(content)
    assert set(msg.vehicles) == {"quad5-betaflight", "truck"}
    assert set(msg.locations) == set(content.locations)
    assert all(isinstance(loc.ground_alt_m, float) for loc in msg.locations.values()), (
        "the page has nobody to ask for a ground left 'auto'"
    )
    # As the server's /api/content: the page's own type takes it.
    ContentMsg.model_validate_json(msg.model_dump_json())


def test_the_simulators_data_has_each_quads_settings_and_airframe() -> None:
    content = load()
    data = webdata.sim_data(content)
    assert set(data["vehicles"]) == {"quad5-betaflight"}
    quad = data["vehicles"]["quad5-betaflight"]
    assert quad["airframe"] == "racer5in"
    for video, grid in (("analog", "NTSC"), ("digital", "HD")):
        lines = quad["settings"][video].splitlines()
        assert "feature GPS" in lines and f"set vcd_video_system = {grid}" in lines
        assert not any("#" in line or not line.strip() for line in lines), "comments are gone"
    frame = airframe("racer5in")
    packed = data["airframes"]["racer5in"]
    assert packed["params"] == pytest.approx(np.asarray(Plant(frame).params).tolist())
    assert packed["mass"] == frame.mass and packed["rotor_max"] == frame.rotor_max


def test_written_out_as_the_page_fetches_it(tmp_path: Path) -> None:
    webdata.write(tmp_path)
    content = json.loads((tmp_path / "content.json").read_text())
    assert "quad5-betaflight" in content["vehicles"]
    config = json.loads((tmp_path / "config.json").read_text())
    assert config["map_keys"] == {"google_maps_api_key": None, "cesium_ion_token": None}, (
        "a public page ships no key"
    )
    assert "racer5in" in json.loads((tmp_path / "sim.json").read_text())["airframes"]
    assert (tmp_path / "osd" / "font" / "betaflight.png").read_bytes().startswith(b"\x89PNG")


@pytest.mark.skipif(CC is None, reason="no C compiler")
def test_the_generated_c_plant_is_the_jax_backends(tmp_path: Path) -> None:
    """web/wasm/sfd.c, compiled here as it is for the page (no fast-math), against the JAX
    backend at the pin the game's physics runs: a frame's step and its IMU, from states all
    over the envelope, and a second of flight."""
    assert CC is not None
    lib_path = tmp_path / "libsfd.so"
    subprocess.run(
        [CC, "-O2", "-fPIC", "-shared", "-o", str(lib_path), str(webdata.WASM / "sfd.c"), "-lm"],
        check=True,
    )
    lib = ctypes.CDLL(str(lib_path))
    vec = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    lib.sfd_advance.argtypes = [vec, vec, vec, ctypes.c_double, ctypes.c_int,
                                ctypes.c_double, ctypes.c_double]
    lib.sfd_advance.restype = None
    lib.sfd_imu.argtypes = [vec, vec, vec, vec, vec, vec]
    lib.sfd_imu.restype = None
    lib.sfd_throttle_to_speed.argtypes = [ctypes.c_double] * 4
    lib.sfd_throttle_to_speed.restype = ctypes.c_double

    frame = airframe("racer5in")
    plant = Plant(frame)
    params = np.ascontiguousarray(np.asarray(plant.params), dtype=np.float64)
    header = (webdata.WASM / "sfd.h").read_text()
    assert f"#define SFD_NP {params.size} " in header and "#define SFD_NS 17 " in header
    zero, eye = np.zeros(3), np.ascontiguousarray(np.eye(3).ravel())

    def advance(s: np.ndarray, u: np.ndarray, dt: float, steps: int) -> tuple[np.ndarray, ...]:
        out, imu = np.ascontiguousarray(s, dtype=np.float64).copy(), np.empty(6)
        lib.sfd_advance(out, np.ascontiguousarray(u), params, dt, steps, 0.0, frame.rotor_max)
        lib.sfd_imu(out, np.ascontiguousarray(u), params, zero, eye, imu)
        return out, imu

    rng = np.random.default_rng(11)
    for _ in range(40):
        q = rng.standard_normal(4)
        s = np.concatenate([
            rng.uniform(-50, 50, 3), rng.uniform(-30, 30, 3), q / np.linalg.norm(q),
            rng.uniform(-15, 15, 3), rng.uniform(0, frame.rotor_max, 4),
        ])
        u = Plant.inputs(rng.uniform(0, frame.rotor_max, 4), tuple(rng.uniform(-8, 8, 3)))
        want, want_imu = plant.advance(s, u, 0.001, 1)
        got, got_imu = advance(s, u, 0.001, 1)
        np.testing.assert_allclose(got, want, rtol=1e-11, atol=1e-9)
        np.testing.assert_allclose(got_imu, want_imu, rtol=1e-9, atol=1e-7)

    # A second from the pad at a climbing throttle: a thousand steps, one after another.
    s_c = s_jax = Plant.rest_state((0.0, 0.0, -5.0), 30.0)
    u = Plant.inputs(np.array([1200.0, 1150.0, 1250.0, 1180.0]), (2.0, -1.0, 0.0))
    for _ in range(1000):
        s_jax, _ = plant.advance(s_jax, u, 0.001, 1)
        s_c, _ = advance(s_c, u, 0.001, 1)
    np.testing.assert_allclose(s_c, s_jax, rtol=1e-8, atol=1e-8)

    curve = sfd_jax.throttle_to_speed_fn()
    for throttle in (0.0, 0.1, 0.5, 1.0):
        want_speed = float(np.asarray(curve(throttle, frame.rotor_min, frame.rotor_max, 0.5)))
        got_speed = lib.sfd_throttle_to_speed(throttle, frame.rotor_min, frame.rotor_max, 0.5)
        assert got_speed == pytest.approx(want_speed, rel=1e-12)
