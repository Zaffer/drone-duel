"""The COP's drone model: swarm.fpvmodel's conversion of the committed Sketchfab download."""

import struct
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from swarm.fpvmodel import (
    ARDUPILOT_QUAD_X,
    build_glb,
    nose_rotation,
    pieces,
    spin_direction,
    subset,
)
from swarm.gltf import ARRAY_BUFFER, GltfWriter, Primitive, read_accessor, read_glb

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets" / "fpv_drone.glb"
COP_MODEL = ROOT / "src" / "swarm" / "web" / "models" / "fpv-drone.glb"


def test_glb_round_trip() -> None:
    gltf = GltfWriter()
    points = np.arange(12, dtype=np.float32).reshape(4, 3)
    index = gltf.accessor(points, "VEC3", ARRAY_BUFFER, True)
    gltf.nodes.append({"name": "root"})
    doc, buffer = read_glb(gltf.to_glb(0, {"generator": "test"}))
    assert doc["asset"] == {"version": "2.0", "generator": "test"}
    np.testing.assert_array_equal(read_accessor(doc, buffer, index), points)


def _blade(pitch: float, flip_winding: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """A two-blade propeller lying along z, chord along x, its edges `pitch` high per metre of
    chord: positive pitch raises the edge that leads when it turns anticlockwise from above."""
    radius = np.linspace(0.1, 1.0, 10)
    chord = np.linspace(-0.1, 0.1, 3)
    points, triangles = [], []
    for side in (1.0, -1.0):
        base = len(points)
        for r in radius:
            for c in chord:
                # Anticlockwise from above, the blade at +z moves towards +x, the one at -z
                # towards -x.
                points.append([c, pitch * c * side, side * r])
        for i in range(len(radius) - 1):
            for j in range(len(chord) - 1):
                a = base + i * len(chord) + j
                b, d, e = a + 1, a + len(chord), a + len(chord) + 1
                triangles += [[a, d, b], [b, d, e]]
    tris = np.array(triangles, dtype=np.uint32)
    if flip_winding:
        tris = tris[:, ::-1]
    return np.array(points, dtype=np.float32), tris.reshape(-1)


def test_spin_direction_follows_the_blade_pitch_whatever_the_winding() -> None:
    for flip in (False, True):
        assert spin_direction(*_blade(0.3, flip)) == 1
        assert spin_direction(*_blade(-0.3, flip)) == -1


def test_pieces_join_across_uv_seams_and_subset_keeps_only_used_vertices() -> None:
    positions = np.array(
        [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0], [1, 1, 0], [5, 5, 5], [6, 5, 5], [5, 6, 5]],
        dtype=np.float32,
    )  # vertex 3 repeats vertex 1, as at a UV seam
    mesh = Primitive("m", {"POSITION": positions}, np.array([0, 1, 2, 3, 4, 2, 5, 6, 7]), None)
    labels = pieces(mesh)
    assert labels[0] == labels[1] != labels[2]
    part = subset(mesh, labels == labels[2], "far")
    np.testing.assert_array_equal(part.attributes["POSITION"], positions[5:])
    np.testing.assert_array_equal(part.indices, [0, 1, 2])


def test_nose_rotation_puts_the_front_between_the_two_hubs_nearest_the_camera() -> None:
    turn = np.radians(25.0)
    hubs = []
    for corner in np.radians([45.0, 135.0, 225.0, 315.0]) + turn:
        hubs.append([np.sin(corner), 0.0, np.cos(corner)])
    camera = (np.sin(turn + np.pi / 2), np.cos(turn + np.pi / 2))
    rotation = nose_rotation(np.array(hubs), camera)
    turned = np.array(hubs) @ rotation.T
    front = turned[turned[:, 2] > 0]
    assert len(front) == 2
    np.testing.assert_allclose(front[:, 0].sum(), 0.0, atol=1e-12)
    np.testing.assert_allclose(sorted(np.abs(front[:, 0])), [np.sqrt(0.5)] * 2)


def _nodes(doc: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {n["name"]: n for n in doc["nodes"]}


@pytest.fixture(scope="module")
def rebuilt() -> tuple[dict[str, Any], bytes]:
    return read_glb(build_glb(SOURCE.read_bytes(), texture_px=None))


def test_model_is_in_metres_on_the_ground_with_the_nose_on_z(
    rebuilt: tuple[dict[str, Any], bytes],
) -> None:
    doc, buffer = rebuilt
    nodes = _nodes(doc)
    hubs = np.array([nodes[name]["translation"] for name in ARDUPILOT_QUAD_X])
    # Front right and rear left mirror front left and rear right: the frame is square to +Z.
    np.testing.assert_allclose(hubs[0], hubs[2] * [-1, 1, 1], atol=1e-4)
    np.testing.assert_allclose(hubs[1], hubs[3] * [-1, 1, 1], atol=1e-4)
    np.testing.assert_allclose(hubs[0], hubs[1] * [-1, 1, -1], atol=1e-4)
    points = []
    for node in doc["nodes"]:
        if "mesh" in node:
            primitive = doc["meshes"][node["mesh"]]["primitives"][0]
            position = read_accessor(doc, buffer, primitive["attributes"]["POSITION"])
            points.append(position + node.get("translation", [0.0, 0.0, 0.0]))
    every = np.vstack(points)
    assert every[:, 1].min() == pytest.approx(0.0, abs=1e-6)
    assert 0.25 < np.ptp(every[:, 0]) < 0.35  # a 5 inch quad
    assert len(points) == 9


def test_props_are_pitched_the_ardupilot_quad_x_way(
    rebuilt: tuple[dict[str, Any], bytes],
) -> None:
    doc, _ = rebuilt
    spins = {n["name"]: n["extras"]["spin"] for n in doc["nodes"] if "extras" in n}
    assert spins == ARDUPILOT_QUAD_X
    assert "animations" not in doc  # the COP spins them at its own rates


def test_cop_model_is_the_converted_source(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    """The committed COP model matches the converter (rebuild it after changing either)."""
    doc, buffer = rebuilt
    served, served_buffer = read_glb(COP_MODEL.read_bytes())
    for key in ("nodes", "meshes", "materials", "textures"):
        assert served[key] == doc[key], key
    for mesh in doc["meshes"]:
        index = mesh["primitives"][0]["attributes"]["POSITION"]
        np.testing.assert_array_equal(
            read_accessor(served, served_buffer, index), read_accessor(doc, buffer, index)
        )


def test_cop_model_textures_are_small() -> None:
    doc, buffer = read_glb(COP_MODEL.read_bytes())
    for image in doc["images"]:
        start = doc["bufferViews"][image["bufferView"]]["byteOffset"]
        assert buffer[start : start + 8] == b"\x89PNG\r\n\x1a\n"
        assert max(struct.unpack(">II", buffer[start + 16 : start + 24])) <= 512
