"""The pilot behind the pad: tools.models.operatormodel's posing of the committed Fab character."""

from pathlib import Path
from typing import Any

import numpy as np
import pytest

from tools.models.fbx import Scene, SkinnedMesh
from tools.models.gltf import read_accessor, read_glb
from tools.models.operatormodel import (
    RADIO_CENTRE,
    WRIST_OUT,
    Pose,
    align,
    build_glb,
    hand_frame,
    hold_radio,
    twist_about,
)

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets" / "quantum_character.fbx"
SERVED = ROOT / "web" / "public" / "models" / "operator.glb"


@pytest.fixture(scope="module")
def scene() -> Scene:
    return Scene(SOURCE.read_bytes())


@pytest.fixture(scope="module")
def meshes(scene: Scene) -> dict[str, SkinnedMesh]:
    return {m.name: m for m in scene.meshes()}


def test_parts_stand_where_blender_puts_them(meshes: dict[str, SkinnedMesh]) -> None:
    """Bounds in cm as Blender 5.2's FBX importer gives them. The jeans and the patches have
    geometric offsets, which the skin's matrices leave out."""
    expected = {
        "Jeans_Tactic": ([-22.1, -0.1, -10.8], [22.1, 110.7, 24.7]),
        "SKM_Patch_Back": ([-12.0, 136.2, -16.0], [12.0, 142.6, -14.5]),
        "SKM_Patch_Flag_Cap": ([-2.8, 178.9, 6.9], [2.8, 182.0, 9.9]),
        "Head": ([-13.7, 140.4, -9.0], [13.7, 181.8, 13.9]),
    }
    for name, (low, high) in expected.items():
        points = meshes[name].points
        np.testing.assert_allclose(points.min(axis=0), low, atol=0.06, err_msg=name)
        np.testing.assert_allclose(points.max(axis=0), high, atol=0.06, err_msg=name)


def test_skin_weights_sum_to_one(meshes: dict[str, SkinnedMesh]) -> None:
    for mesh in meshes.values():
        total = np.zeros(len(mesh.points))
        for indices, weights in mesh.weights.values():
            np.add.at(total, indices, weights)
        np.testing.assert_allclose(total, 1.0, atol=1e-3, err_msg=mesh.name)


def test_palms_face_the_thighs_in_the_a_pose(scene: Scene) -> None:
    pose = Pose(scene.bones())
    assert hand_frame(pose, "l")[0, 1] < -0.5
    assert hand_frame(pose, "r")[0, 1] > 0.5


def test_twist_is_the_rotation_about_the_axis() -> None:
    axis = np.array([0.0, 0.0, 1.0])
    roll = align(np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]))  # 90° about +Z
    assert twist_about(roll, axis) == pytest.approx(np.pi / 2)
    swing = align(np.array([0.0, 0.0, 1.0]), np.array([0.0, 1.0, 0.0]))  # about X only
    assert twist_about(swing, axis) == pytest.approx(0.0, abs=1e-9)


def test_hands_hold_the_radio_and_bones_keep_their_lengths(scene: Scene) -> None:
    bones = scene.bones()
    pose = Pose(bones)
    hold_radio(pose)
    for side, out in (("l", 1.0), ("r", -1.0)):
        wrist = pose.at(f"hand_{side}")
        assert wrist[0] == pytest.approx(out * WRIST_OUT, abs=0.5)
        assert np.linalg.norm(wrist - RADIO_CENTRE) < 15.0
        for child, parent in (("lowerarm", "upperarm"), ("hand", "lowerarm")):
            posed = np.linalg.norm(pose.at(f"{child}_{side}") - pose.at(f"{parent}_{side}"))
            bind = np.linalg.norm(
                bones[f"{child}_{side}"].bind[:3, 3] - bones[f"{parent}_{side}"].bind[:3, 3]
            )
            assert posed == pytest.approx(bind, abs=1e-6)
        # Fingers round the radio: the middle fingertip is beside or behind it, not in front.
        tip = pose.at(f"middle_03_{side}")
        assert abs(tip[0]) < WRIST_OUT + 3.0
        assert tip[2] > RADIO_CENTRE[2] - 3.0


@pytest.fixture(scope="module")
def rebuilt() -> tuple[dict[str, Any], bytes]:
    return read_glb(build_glb(SOURCE.read_bytes()))


def _points(doc: dict[str, Any], buffer: bytes, name: str | None = None) -> np.ndarray:
    return np.vstack(
        [
            read_accessor(doc, buffer, p["attributes"]["POSITION"])
            for mesh in doc["meshes"]
            if name is None or mesh["name"] == name
            for p in mesh["primitives"]
        ]
    )


def test_model_is_a_person_in_metres_on_the_ground(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    doc, buffer = rebuilt
    every = _points(doc, buffer)
    assert every[:, 1].min() == pytest.approx(0.0, abs=1e-6)
    assert 1.75 < every[:, 1].max() < 1.9
    assert np.ptp(every[:, 0]) < 0.7  # arms in, not the A-pose's span


def test_model_faces_z_with_its_left_on_x(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    """Goggles and radio in front (+Z), the holster on the right thigh (glTF -X)."""
    doc, buffer = rebuilt
    assert _points(doc, buffer, "goggles")[:, 2].min() > 0.08
    assert _points(doc, buffer, "radio")[:, 2].min() > 0.2
    assert _points(doc, buffer, "M_Holster_Hard")[:, 0].mean() < -0.1


def test_every_mesh_has_a_material_of_its_own(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    doc, _ = rebuilt
    names = [m["name"] for m in doc["meshes"]]
    assert len(names) == len(set(names))
    for mesh in doc["meshes"]:
        assert doc["materials"][mesh["primitives"][0]["material"]]["name"] == mesh["name"]
    assert {"goggles", "radio", "M_Head", "M_Cap"} <= set(names)
    assert not {"M_Teeth", "M_Eyeball", "M_Sclera"} & set(names)


def test_served_model_is_the_converted_source(rebuilt: tuple[dict[str, Any], bytes]) -> None:
    """The committed model matches the converter (rebuild it after changing either)."""
    doc, buffer = rebuilt
    served, served_buffer = read_glb(SERVED.read_bytes())
    for key in ("nodes", "meshes", "materials"):
        assert served[key] == doc[key], key
    np.testing.assert_array_equal(_points(served, served_buffer), _points(doc, buffer))
