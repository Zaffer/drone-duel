"""The ground shooter's rifle: tools.models.riflemodel's conversion of the committed Fab rifle,
held in the operator's character's arms."""

import io
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from PIL import Image
from scipy.spatial import KDTree

from tools.models.fbx import Scene
from tools.models.gltf import read_accessor, read_glb
from tools.models.riflemodel import EYE_RELIEF, build_glb
from tools.models.rifletextures import SIDE_PX, SOURCES

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets" / "sm_rifle.fbx"
CHARACTER = ROOT / "assets" / "quantum_character.fbx"
TEXTURES = ROOT / "assets" / "rifle_textures"
SERVED = ROOT / "web" / "public" / "models" / "rifle.glb"


def test_static_mesh_stands_where_blender_puts_it() -> None:
    """Bounds in cm as Blender 5.2's FBX importer gives them (its Z up is the file's Y): the
    model's own transform (a 0.97 scale, and rotations that cancel) is applied."""
    (mesh,) = Scene(SOURCE.read_bytes()).meshes()
    np.testing.assert_allclose(mesh.points.min(axis=0), [-2.6, -8.3, -27.2], atol=0.1)
    np.testing.assert_allclose(mesh.points.max(axis=0), [2.9, 22.0, 57.0], atol=0.1)


@pytest.fixture(scope="module")
def rebuilt() -> tuple[dict[str, Any], bytes]:
    return read_glb(build_glb(SOURCE.read_bytes(), CHARACTER.read_bytes(), TEXTURES))


def _nodes(doc: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {n["name"]: n for n in doc["nodes"]}


def _points(doc: dict[str, Any], buffer: bytes, name: str) -> np.ndarray:
    """A node's points in its parent's frame (only the magazine node is moved)."""
    node = _nodes(doc)[name]
    mesh = doc["meshes"][node["mesh"]]
    points = np.vstack(
        [read_accessor(doc, buffer, p["attributes"]["POSITION"]) for p in mesh["primitives"]]
    )
    return points + node.get("translation", [0.0, 0.0, 0.0])


def _extras(doc: dict[str, Any]) -> dict[str, Any]:
    return {
        k: np.array(v) if isinstance(v, list) else v for k, v in doc["scenes"][0]["extras"].items()
    }


def test_nodes_and_their_hierarchy(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    doc, _ = rebuilt
    nodes = _nodes(doc)
    (root,) = doc["scenes"][0]["nodes"]
    assert doc["nodes"][root]["name"] == "shooter"

    def names(node: dict[str, Any]) -> set[str]:
        return {doc["nodes"][i]["name"] for i in node.get("children", [])}

    assert names(doc["nodes"][root]) == {
        "rifle",
        "holo",
        "magazine",
        "arm_right",
        "arm_left_grip",
    }
    assert names(nodes["holo"]) == {"holo_glass"}
    assert names(nodes["magazine"]) == {"arm_left_mag"}
    np.testing.assert_allclose(nodes["magazine"]["translation"], _extras(doc)["mag_well"])
    glass = doc["materials"][
        doc["meshes"][nodes["holo_glass"]["mesh"]]["primitives"][0]["material"]
    ]
    assert glass["alphaMode"] == "BLEND"


def test_rifle_points_the_bore_down_z_from_the_butt(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    """Origin at the butt pad, muzzle on +Z, the sight on top, the magazine and grip below, and
    the ejection port on the shooter's right (-X)."""
    doc, buffer = rebuilt
    x = _extras(doc)
    rifle = _points(doc, buffer, "rifle")
    assert rifle[:, 2].min() == pytest.approx(0.0, abs=1e-6)
    np.testing.assert_allclose(x["butt"], 0.0)
    np.testing.assert_allclose(x["muzzle"], x["bore"][1])
    assert x["muzzle"][2] == pytest.approx(rifle[:, 2].max(), abs=1e-4)
    assert 0.8 < x["length"] < 0.9  # a short-barrelled carbine
    assert x["bore"][0][2] < x["bore"][1][2]
    assert abs(x["bore"][0][0]) < 0.003 and np.ptp(x["bore"][:, :2], axis=0).max() < 1e-6
    assert _points(doc, buffer, "holo")[:, 1].min() > x["bore"][0][1]
    assert _points(doc, buffer, "magazine")[:, 1].max() < x["bore"][0][1]
    assert x["grip"][1] < 0.0 and x["grip"][2] < x["mag_well"][2]
    assert x["ejection_port"][0] < -0.015 and x["ejection_dir"][0] == -1.0
    assert x["bore"][0][2] - 0.06 < x["ejection_port"][2] < x["bore"][0][2]


def test_sight_picture(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    """The eye on the sight's axis behind it, the axis level with the bore and a holographic
    sight's height above it, the window round the axis, the front post lower in the window."""
    doc, _ = rebuilt
    x = _extras(doc)
    rear, front = x["holo_axis"]
    np.testing.assert_allclose(rear[:2], front[:2])
    np.testing.assert_allclose(x["eye"][:2], rear[:2])
    assert x["eye"][2] == pytest.approx(rear[2] - EYE_RELIEF)
    assert 0.06 < x["sight_height"] < 0.07
    assert x["sight_height"] == pytest.approx(rear[1] - x["bore"][0][1], abs=1e-4)
    window = x["holo_window"]
    assert rear[2] < window[0][2] < front[2]
    assert window[:, 0].min() < rear[0] < window[:, 0].max()
    assert window[:, 1].min() < rear[1] < window[:, 1].max()
    assert window[0][0] > window[1][0]  # the first corner is the top left, on +X
    post, aperture = x["front_post_tip"], x["rear_aperture"]
    assert x["bore"][0][1] < post[1] < window[:, 1].min()
    assert post[2] > 0.7 and post[1] == aperture[1] and aperture[2] < window[0][2]


def test_magazine_drops_out_along_its_axis(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    doc, buffer = rebuilt
    x = _extras(doc)
    axis = x["mag_axis"]
    assert np.linalg.norm(axis) == pytest.approx(1.0, abs=1e-4)
    assert axis[1] < -0.99  # an AR's well is square to the bore
    magazine = _points(doc, buffer, "magazine") - x["mag_well"]
    assert magazine[:, 1].max() == pytest.approx(0.0, abs=1e-4)  # the seat is its top
    assert np.abs(magazine @ axis).max() > 0.15


def test_no_arm_in_the_sight_or_before_the_eye(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    """Nothing of the arms in the view through the window, out past the muzzle, nor within a
    hand's breadth of the eye on its side of the face."""
    doc, buffer = rebuilt
    x = _extras(doc)
    eye, window = x["eye"], x["holo_window"]
    for name in ("arm_right", "arm_left_grip"):
        points = _points(doc, buffer, name) - eye
        ahead = points[:, 2] > 0.0
        scale = (window[0][2] - eye[2]) / points[ahead, 2]
        at_window = points[ahead, :2] * scale[:, None] + eye[:2]
        inside = (
            (at_window[:, 0] > window[:, 0].min())
            & (at_window[:, 0] < window[:, 0].max())
            & (at_window[:, 1] > window[:, 1].min())
            & (at_window[:, 1] < window[:, 1].max())
        )
        assert not inside.any(), name
        assert np.linalg.norm(points[ahead], axis=1).min() > 0.08, name


def test_shoulders_where_a_shooters_are(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    """The right sleeve goes back past the butt to the shoulder; the left reaches out left of
    the stock to its shoulder; the hands are on the grip, the handguard and the magazine."""
    doc, buffer = rebuilt
    x = _extras(doc)
    right = _points(doc, buffer, "arm_right")
    assert right[:, 2].min() < -0.03 and right[:, 0].max() < 0.04
    left = _points(doc, buffer, "arm_left_grip")
    assert left[:, 0].max() > 0.2 and left[:, 2].max() > 0.6
    assert np.linalg.norm(right - x["grip"], axis=1).min() < 0.03
    handguard = _points(doc, buffer, "rifle")
    handguard = handguard[handguard[:, 2] > x["bore"][0][2]]
    assert KDTree(handguard).query(left)[0].min() < 0.003
    magazine = _points(doc, buffer, "magazine")
    on_mag = _points(doc, buffer, "arm_left_mag") + x["mag_well"]  # the magazine's child
    assert KDTree(magazine).query(on_mag)[0].min() < 0.003


def test_materials_are_textured(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    doc, buffer = rebuilt
    names = {m["name"] for m in doc["materials"]}
    assert names == {*SOURCES, "holo_glass"}
    for material in doc["materials"]:
        if material["name"] in SOURCES:
            assert {"normalTexture", "occlusionTexture"} <= material.keys()
    largest = max(max(sides.values()) for sides in SIDE_PX.values())
    for image in doc["images"]:
        view = doc["bufferViews"][image["bufferView"]]
        start = view["byteOffset"]
        size = Image.open(io.BytesIO(buffer[start : start + view["byteLength"]])).size
        assert max(size) <= largest


def test_committed_textures_are_every_map_at_its_size() -> None:
    for material, sides in SIDE_PX.items():
        for kind, side in sides.items():
            image = Image.open(TEXTURES / f"{material}_{kind}.jpg")
            assert image.size == (side, side), (material, kind)


def test_served_model_is_the_converted_source(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    """The committed model matches the converter (rebuild it after changing either)."""
    doc, buffer = rebuilt
    served, served_buffer = read_glb(SERVED.read_bytes())
    for key in ("nodes", "meshes", "materials", "textures", "scenes"):
        assert served[key] == doc[key], key
    for name in _nodes(doc):
        if "mesh" in _nodes(doc)[name]:
            np.testing.assert_array_equal(
                _points(served, served_buffer, name), _points(doc, buffer, name)
            )
    assert SERVED.stat().st_size < 8e6
