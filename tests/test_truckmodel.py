"""The COP's point B truck: swarm.truckmodel's conversion of the committed Sketchfab download."""

import io
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from PIL import Image

from swarm.gltf import Primitive, read_accessor, read_glb
from swarm.truckmodel import GLASS_NODES, build_glb, merge, truck_parts

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets" / "military_truck.glb"
COP_MODEL = ROOT / "src" / "swarm" / "web" / "models" / "military-truck.glb"


def test_merge_offsets_each_parts_indices_past_the_vertices_before_it() -> None:
    a = Primitive("a", {"POSITION": np.zeros((3, 3), np.float32)}, np.array([0, 1, 2]), 0)
    b = Primitive("b", {"POSITION": np.ones((4, 3), np.float32)}, np.array([0, 2, 3]), 0)
    merged = merge([a, b], "ab")
    assert merged.attributes["POSITION"].shape == (7, 3)
    np.testing.assert_array_equal(merged.indices, [0, 1, 2, 3, 5, 6])
    assert merged.indices.dtype == np.uint32


@pytest.fixture(scope="module")
def parts() -> dict[str, Primitive]:
    doc, buffer = read_glb(SOURCE.read_bytes())
    return {p.name: p for p in truck_parts(doc, buffer)}


def _centre(part: Primitive) -> np.ndarray:
    points = part.attributes["POSITION"]
    return (points.min(axis=0) + points.max(axis=0)) / 2


def test_truck_faces_z_and_drives_on_the_left(parts: dict[str, Primitive]) -> None:
    """Front on +Z, glTF +X its left: headlights ahead, tail lights behind, and the steering
    wheel on the left, as in a left-hand drive truck."""
    assert _centre(parts["headlights_low.001"])[2] > 2.0
    assert _centre(parts["backlights_low.001"])[2] < -2.0
    assert _centre(parts["wheel_low.001"])[0] > 0.3


@pytest.fixture(scope="module")
def rebuilt() -> tuple[dict[str, Any], bytes]:
    return read_glb(build_glb(SOURCE.read_bytes(), texture_px=None))


def _points(doc: dict[str, Any], buffer: bytes) -> np.ndarray:
    return np.vstack(
        [
            read_accessor(doc, buffer, p["attributes"]["POSITION"])
            for mesh in doc["meshes"]
            for p in mesh["primitives"]
        ]
    )


def test_model_is_a_truck_in_metres_on_the_ground(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    doc, buffer = rebuilt
    every = _points(doc, buffer)
    assert every[:, 1].min() == pytest.approx(0.0, abs=1e-6)
    low, high = every.min(axis=0), every.max(axis=0)
    np.testing.assert_allclose((low + high)[[0, 2]] / 2, [0.0, 0.0], atol=1e-6)
    length, height, width = np.ptp(every[:, 2]), np.ptp(every[:, 1]), np.ptp(every[:, 0])
    assert 4.5 < length < 5.5
    assert 1.5 < height < 2.0
    assert 2.0 < width < 2.5  # mirrors included


def test_only_the_glass_is_blended(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    doc, _ = rebuilt
    blended = {m["name"] for m in doc["materials"] if m.get("alphaMode") == "BLEND"}
    assert blended == {"glass"}
    meshes = {mesh["name"]: mesh["primitives"][0] for mesh in doc["meshes"]}
    assert doc["materials"][meshes["glass"]["material"]]["name"] == "glass"
    assert len(meshes) == 4  # three materials and the glass
    assert len(GLASS_NODES) == 5


def test_cop_model_is_the_converted_source(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    """The committed COP model matches the converter (rebuild it after changing either)."""
    doc, buffer = rebuilt
    served, served_buffer = read_glb(COP_MODEL.read_bytes())
    for key in ("nodes", "meshes", "materials", "textures"):
        assert served[key] == doc[key], key
    np.testing.assert_array_equal(_points(served, served_buffer), _points(doc, buffer))


def test_cop_model_textures_are_small() -> None:
    doc, buffer = read_glb(COP_MODEL.read_bytes())
    for image in doc["images"]:
        view = doc["bufferViews"][image["bufferView"]]
        start = view["byteOffset"]
        data = buffer[start : start + view["byteLength"]]
        assert max(Image.open(io.BytesIO(data)).size) <= 512
