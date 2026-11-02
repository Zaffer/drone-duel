"""The drone operator: Fab's "Quantum Modular Character Free Sample" (Standard License) posed as
an FPV pilot, goggles on and a radio in both hands.

assets/quantum_character.fbx is the listing's SKM_Character.fbx as it came (fbx_all_assets.zip):
the whole soldier, skinned to a UE5 mannequin skeleton, in centimetres with +Y up and the face
toward +Z (glTF's own frame), standing in an A-pose. After changing this converter, rebuild the
model the page serves with

    uv run python -m tools.models.operatormodel assets/quantum_character.fbx \\
        web/public/models/operator.glb

The arms reach the radio by two-bone IK, the wrists' roll is shared out along the forearms'
twist bones, and the fingers curl round the radio's sides. The skin is baked in that pose into a
static mesh, one per material, in metres with the feet on y = 0. The cap is turned back to front
to clear the goggles. The eyes, lashes and teeth are left out: the goggles cover the eyes.

The listing's textures (pbr_textures.zip, 1.1 GB) are not used: each material is a flat colour
picked to match its texture's name (the shirt's and jeans' blue, the pouches' green).
"""

import sys
from pathlib import Path

import numpy as np

from tools.models.fbx import Bone, FbxFormatError, Scene, SkinnedMesh
from tools.models.gltf import GltfWriter, Primitive, unit

SOURCE_URL = "https://www.fab.com/listings/8e200050-3158-4762-b297-f785b5b1533d"
LEFT_OUT = {"Eye", "Eyelashes", "Teeth_UP", "Teeth_Down"}
"""Under the goggles, or inside the mouth."""
BACK_TO_FRONT = {"Cap", "SKM_Patch_Flag_Cap"}
"""Turned about the crown: the brim would cut through the goggles."""
COLOURS: dict[str, tuple[str, float]] = {
    # material: sRGB colour, roughness
    "M_Head": ("#c49a7c", 0.7),
    "M_Quantum_Arms": ("#c49a7c", 0.7),
    "M_Shirt_RolledUp": ("#4a5a72", 0.95),
    "M_Jeans": ("#36425a", 0.95),
    "M_Bulletproof_Light": ("#7d7259", 0.9),
    "M_Drops_Tactical": ("#565c3c", 0.9),
    "M_Cap": ("#2e3236", 0.9),
    "M_Patches": ("#76776e", 0.9),
    "M_Holster_Hard": ("#262729", 0.6),
    "M_Pistol": ("#1d1e20", 0.5),
    "goggles": ("#1a1b1f", 0.4),
    "strap": ("#25262b", 0.9),
    "radio": ("#2c2e33", 0.5),
    "sticks": ("#111214", 0.4),
}

# The pose, in the source's centimetres. The radio is held at the belly, its face tilted up
# toward the goggles; the hands grip its sides, thumbs toward the face.
RADIO_CENTRE = np.array([0.0, 118.0, 30.0])
RADIO_TILT_DEG = 40.0
"""Its top leans away from the pilot by this much from upright."""
RADIO_SIZE = np.array([19.0, 12.0, 5.5])
"""Width, height and depth."""
WRIST_OUT = 12.0
"""How far each wrist is from the radio's middle, sideways."""
ELBOW_POLE = np.array([0.5, -1.0, -0.2])
"""Where the elbows point, the left's (+X); the right mirrors it."""
FINGER_CURL_DEG = (50.0, 60.0, 30.0)
"""Each finger's three joints, knuckle first."""
GOGGLES_SIZE = np.array([17.0, 7.5, 7.0])
GOGGLES_GAP = 0.6
"""Between the goggles' back and the face, and the strap and the head."""


class Pose:
    """The skeleton's global transforms, starting in the bind pose."""

    def __init__(self, bones: dict[str, Bone]) -> None:
        self.bones = bones
        self.posed = {name: b.bind.copy() for name, b in bones.items()}
        self.below: dict[str, list[str]] = {name: [] for name in bones}
        for b in bones.values():
            if b.parent is not None:
                self.below[b.parent].append(b.name)

    def at(self, name: str) -> np.ndarray:
        return self.posed[name][:3, 3].copy()

    def rotate(self, name: str, rotation: np.ndarray) -> None:
        """Turn bone `name` and everything below it by `rotation` (global) about its joint."""
        pivot = self.at(name)
        turn = np.eye(4)
        turn[:3, :3] = rotation
        turn[:3, 3] = pivot - rotation @ pivot
        stack = [name]
        while stack:
            b = stack.pop()
            self.posed[b] = turn @ self.posed[b]
            stack.extend(self.below[b])

    def skinning(self) -> dict[str, np.ndarray]:
        """Each bone's skinning matrix: from its bind pose to its pose now."""
        return {n: self.posed[n] @ np.linalg.inv(b.bind) for n, b in self.bones.items()}


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def align(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """The smallest rotation turning direction `a` onto direction `b`."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    axis = np.cross(a, b)
    if np.linalg.norm(axis) < 1e-9:
        return np.eye(3)
    return axis_angle(axis, float(np.arctan2(np.linalg.norm(axis), a @ b)))


def twist_about(rotation: np.ndarray, axis: np.ndarray) -> float:
    """The angle of `rotation`'s twist about `axis` (its swing-twist decomposition)."""
    w = np.sqrt(max(0.0, 1 + np.trace(rotation))) / 2
    v = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    ) / (4 * max(w, 1e-9))
    return 2 * float(np.arctan2(v @ (axis / np.linalg.norm(axis)), w))


def hand_frame(pose: Pose, side: str) -> np.ndarray:
    """Columns: along the hand toward the fingers, the palm's normal, and their cross."""
    along = unit((pose.at(f"middle_01_{side}") - pose.at(f"hand_{side}"))[None])[0]
    across = pose.at(f"index_01_{side}") - pose.at(f"pinky_01_{side}")
    across = unit((across - (across @ along) * along)[None])[0]
    # The index finger is on the thumb's side, so on the left hand the palm is along × across
    # (hanging in the A-pose, thumb forward, it faces the thigh); the right mirrors it.
    palm = np.cross(along, across) * (1.0 if side == "l" else -1.0)
    return np.column_stack([along, palm, np.cross(along, palm)])


def radio_axes() -> np.ndarray:
    """Columns: the radio's right (the pilot's -X), up and face normal (toward the pilot)."""
    tilt = np.radians(RADIO_TILT_DEG)
    up = np.array([0.0, np.cos(tilt), np.sin(tilt)])
    face = np.array([0.0, np.sin(tilt), -np.cos(tilt)])
    return np.column_stack([[-1.0, 0.0, 0.0], up, face])


def hold_radio(pose: Pose) -> None:
    axes = radio_axes()
    up, face = axes[:, 1], axes[:, 2]
    for side, out in (("l", 1.0), ("r", -1.0)):
        sideways = np.array([out, 0.0, 0.0])
        wrist = RADIO_CENTRE + WRIST_OUT * sideways + 1.5 * face - 4.0 * up
        shoulder, elbow, hand = (pose.at(f"{b}_{side}") for b in ("upperarm", "lowerarm", "hand"))
        upper, lower = np.linalg.norm(elbow - shoulder), np.linalg.norm(hand - elbow)
        reach = wrist - shoulder
        d = min(np.linalg.norm(reach), upper + lower - 1e-3)
        toward = reach / np.linalg.norm(reach)
        pole = ELBOW_POLE * np.array([out, 1.0, 1.0])
        pole = unit((pole - (pole @ toward) * toward)[None])[0]
        bend = np.arccos(np.clip((upper**2 + d**2 - lower**2) / (2 * upper * d), -1, 1))
        new_elbow = shoulder + upper * (np.cos(bend) * toward + np.sin(bend) * pole)
        pose.rotate(f"upperarm_{side}", align(elbow - shoulder, new_elbow - shoulder))
        elbow, hand = pose.at(f"lowerarm_{side}"), pose.at(f"hand_{side}")
        pose.rotate(f"lowerarm_{side}", align(hand - elbow, wrist - elbow))

        # The hand: its heel at the radio's lower corner, palm against the side, fingers up
        # the side and round the back, so the thumb (toward the face) comes over the sticks.
        along = unit((up - 0.6 * face)[None])[0]
        palm = -sideways
        target = np.column_stack([along, palm, np.cross(along, palm)])
        rotation = target @ hand_frame(pose, side).T
        forearm = pose.at(f"hand_{side}") - pose.at(f"lowerarm_{side}")
        roll = twist_about(rotation, forearm)
        pose.rotate(f"hand_{side}", rotation)
        for bone, share in (
            (f"lowerarm_twist_01_{side}", 2 / 3),
            (f"lowerarm_twist_02_{side}", 1 / 3),
        ):
            pose.rotate(bone, axis_angle(forearm, roll * share))

        frame = hand_frame(pose, side)
        curl_axis = np.cross(frame[:, 0], frame[:, 1])
        for finger in ("index", "middle", "ring", "pinky"):
            for joint, degrees in zip(("01", "02", "03"), FINGER_CURL_DEG, strict=True):
                pose.rotate(f"{finger}_{joint}_{side}", axis_angle(curl_axis, np.radians(degrees)))


def turn_back_to_front(mesh: SkinnedMesh, crown: np.ndarray) -> None:
    """Half a turn about the vertical through `crown` (x, z), in place."""
    mesh.points[:, [0, 2]] = 2 * crown - mesh.points[:, [0, 2]]
    mesh.normals[..., [0, 2]] *= -1


def bake(mesh: SkinnedMesh, skinning: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """The mesh's points and corner normals in the pose (linear blend skinning)."""
    points = np.zeros_like(mesh.points)
    linear = np.zeros((len(mesh.points), 3, 3))
    for bone, (indices, weights) in mesh.weights.items():
        m = skinning[bone]
        points[indices] += weights[:, None] * (mesh.points[indices] @ m[:3, :3].T + m[:3, 3])
        linear[indices] += weights[:, None, None] * m[:3, :3]
    normals = np.einsum("tcij,tcj->tci", linear[mesh.triangles], mesh.normals)
    return points, unit(normals.reshape(-1, 3)).reshape(normals.shape)


def indexed(points: np.ndarray, triangles: np.ndarray, normals: np.ndarray, name: str) -> Primitive:
    """Triangles whose corners share a point and a normal share a vertex."""
    corners = triangles.reshape(-1)
    key = np.column_stack([corners, np.round(normals.reshape(-1, 3), 3)])
    _, first, inverse = np.unique(key, axis=0, return_index=True, return_inverse=True)
    return Primitive(
        name,
        {
            "POSITION": points[corners[first]].astype(np.float32),
            "NORMAL": normals.reshape(-1, 3)[first].astype(np.float32),
        },
        inverse.reshape(-1).astype(np.uint32),
        None,
    )


def box(centre: np.ndarray, axes: np.ndarray, size: np.ndarray, name: str) -> Primitive:
    """A box of `size` along the columns of `axes`, four vertices a face for flat shading."""
    positions, normals, indices = [], [], []
    for a in range(3):
        for sign in (1.0, -1.0):
            n = axes[:, a] * sign
            u, v = axes[:, (a + 1) % 3], axes[:, (a + 2) % 3]
            su, sv = size[(a + 1) % 3] / 2, size[(a + 2) % 3] / 2
            mid = centre + n * size[a] / 2
            base = len(positions)
            for cu, cv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                positions.append(mid + cu * su * u + cv * sv * v)
                normals.append(n)
            quad = [0, 1, 2, 0, 2, 3] if sign > 0 else [0, 2, 1, 0, 3, 2]
            indices.extend(base + i for i in quad)
    return Primitive(
        name,
        {"POSITION": np.array(positions, np.float32), "NORMAL": np.array(normals, np.float32)},
        np.array(indices, np.uint32),
        None,
    )


def strap(head: np.ndarray, low: float, high: float, segments: int = 48) -> Primitive:
    """A band round the head's points between heights `low` and `high`, just clear of them."""
    band = head[(head[:, 1] > low) & (head[:, 1] < high)]
    centre = (band[:, [0, 2]].min(axis=0) + band[:, [0, 2]].max(axis=0)) / 2
    offset = band[:, [0, 2]] - centre
    angle = np.arctan2(offset[:, 1], offset[:, 0])
    distance = np.linalg.norm(offset, axis=1)
    positions, normals, indices = [], [], []
    for i in range(segments):
        a = 2 * np.pi * i / segments
        near = np.abs((angle - a + np.pi) % (2 * np.pi) - np.pi) < 2 * np.pi / segments
        r = (distance[near].max() if near.any() else distance.max()) + GOGGLES_GAP
        direction = np.array([np.cos(a), 0.0, np.sin(a)])
        for y in (low, high):
            positions.append([centre[0] + r * direction[0], y, centre[1] + r * direction[2]])
            normals.append(direction)
        j, k = 2 * i, 2 * ((i + 1) % segments)
        indices.extend([j, k, k + 1, j, k + 1, j + 1])
    return Primitive(
        "strap",
        {"POSITION": np.array(positions, np.float32), "NORMAL": np.array(normals, np.float32)},
        np.array(indices, np.uint32),
        None,
    )


def gear(eyes: np.ndarray, head: np.ndarray) -> list[Primitive]:
    """The goggles, their strap and the radio. `eyes`: the eyeballs' points; `head`: the posed
    head's. The goggles sit on the bridge of the nose, just in front of the eyes."""
    eye_y = float(eyes[:, 1].mean())
    eye_front = float(eyes[:, 2].max())
    centre = np.array([0.0, eye_y, eye_front + GOGGLES_GAP + GOGGLES_SIZE[2] / 2])
    goggles = box(centre, np.eye(3), GOGGLES_SIZE, "goggles")
    band = strap(head, eye_y - 1.5, eye_y + 1.5)

    axes = radio_axes()
    radio = box(RADIO_CENTRE, axes, RADIO_SIZE, "radio")
    sticks = []
    for across in (-5.0, 5.0):
        at = RADIO_CENTRE + axes[:, 0] * across + axes[:, 1] * 1.5 + axes[:, 2] * 3.5
        sticks.append(box(at, axes, np.array([0.8, 0.8, 2.0]), "sticks"))
    antenna = RADIO_CENTRE + axes[:, 1] * (RADIO_SIZE[1] / 2 + 3.0) + axes[:, 0] * 5.0
    sticks.append(box(antenna, axes, np.array([1.4, 6.0, 1.4]), "sticks"))
    return [goggles, band, radio, *sticks]


def merge(parts: list[Primitive], name: str) -> Primitive:
    offsets = np.cumsum([0] + [len(p.attributes["POSITION"]) for p in parts[:-1]])
    return Primitive(
        name,
        {k: np.concatenate([p.attributes[k] for p in parts]) for k in ("POSITION", "NORMAL")},
        np.concatenate([p.indices + o for p, o in zip(parts, offsets, strict=True)]).astype(
            np.uint32
        ),
        None,
    )


def linear_rgb(hex_colour: str) -> list[float]:
    srgb = np.array([int(hex_colour[i : i + 2], 16) / 255 for i in (1, 3, 5)])
    linear = np.where(srgb <= 0.04045, srgb / 12.92, ((srgb + 0.055) / 1.055) ** 2.4)
    return [round(float(c), 4) for c in linear]


def operator_parts(source: bytes) -> list[Primitive]:
    """The posed operator and gear, one primitive per material, in metres, feet on y = 0."""
    scene = Scene(source)
    pose = Pose(scene.bones())
    hold_radio(pose)
    skinning = pose.skinning()
    every = {m.name: m for m in scene.meshes()}
    if missing := {"Head", "Cap", "Eye"} - every.keys():
        raise FbxFormatError(f"no {sorted(missing)} mesh")
    eyes = every["Eye"].points  # the head bone stays put: bind pose and pose agree
    meshes = [m for name, m in every.items() if name not in LEFT_OUT]

    top = every["Head"].points
    top = top[top[:, 1] > top[:, 1].max() - 10.0][:, [0, 2]]
    crown = (top.min(axis=0) + top.max(axis=0)) / 2
    groups: dict[str, list[Primitive]] = {}
    head: np.ndarray | None = None
    for mesh in meshes:
        if mesh.name in BACK_TO_FRONT:
            turn_back_to_front(mesh, crown)
        points, normals = bake(mesh, skinning)
        if mesh.name == "Head":
            head = points
        materials = np.array(mesh.materials)
        for material in np.unique(materials):
            if material not in COLOURS:
                raise FbxFormatError(f"{mesh.name}: no colour for {material}")
            keep = materials == material
            part = indexed(points, mesh.triangles[keep], normals[keep], material)
            groups.setdefault(material, []).append(part)
    assert head is not None

    for part in gear(eyes, head):
        groups.setdefault(part.name, []).append(part)

    out = []
    for material, members in groups.items():
        merged = merge(members, material)
        merged.attributes["POSITION"] *= np.float32(0.01)
        out.append(merged)
    floor = min(float(p.attributes["POSITION"][:, 1].min()) for p in out)
    for p in out:
        p.attributes["POSITION"][:, 1] -= np.float32(floor)
    return out


def build_glb(source: bytes) -> bytes:
    gltf = GltfWriter()
    children = []
    for part in operator_parts(source):
        colour, roughness = COLOURS[part.name]
        gltf.materials.append(
            {
                "name": part.name,
                "pbrMetallicRoughness": {
                    "baseColorFactor": [*linear_rgb(colour), 1.0],
                    "metallicFactor": 0.0,
                    "roughnessFactor": roughness,
                },
                **({"doubleSided": True} if part.name == "strap" else {}),
            }
        )
        part.material = len(gltf.materials) - 1
        children.append(gltf.mesh_node(part))
    root = len(gltf.nodes)
    gltf.nodes.append({"name": "operator", "children": children})
    asset = {
        "generator": "tools.models.operatormodel",
        "copyright": f'"Quantum Modular Character Free Sample", Fab Standard License, {SOURCE_URL}',
    }
    return gltf.to_glb(root, asset)


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: python -m tools.models.operatormodel <SKM_Character.fbx> <out.glb>"
        )
    out = Path(sys.argv[2])
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(build_glb(Path(sys.argv[1]).read_bytes()))
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
