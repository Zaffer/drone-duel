"""The drone operator: Fab's "Quantum Modular Character Free Sample" (Standard License) posed as
an FPV pilot, goggles on and a radio in both hands.

assets/quantum_character.fbx is the listing's SKM_Character.fbx as it came (fbx_all_assets.zip):
the whole soldier, skinned to a UE5 mannequin skeleton, in centimetres with +Y up and the face
toward +Z (glTF's own frame), standing in an A-pose. After changing this converter, rebuild the
model the page serves with

    uv run python -m tools.models.operatormodel assets/quantum_character.fbx \\
        web/public/models/operator.glb

It reads the textures from assets/quantum_textures/, beside the source
(tools.models.operatortextures).

The arms reach the radio by two-bone IK, the wrists' roll is shared out along the forearms'
twist bones, and the fingers curl round the radio's sides. The skin is baked in that pose into a
static mesh, one per material, in metres with the feet on y = 0. The cap is turned back to front
to clear the goggles. The eyes, lashes and teeth are left out: the goggles cover the eyes.

Each of the character's materials gets its base colour, normal and occlusion-roughness-metal
maps; the gear (goggles, radio, antennas) is flat colours.
"""

import sys
from pathlib import Path

import numpy as np

from tools.models.fbx import Bone, FbxFormatError, Scene, SkinnedMesh
from tools.models.gltf import GltfWriter, Primitive, unit
from tools.models.operatortextures import SOURCES as TEXTURED

SOURCE_URL = "https://www.fab.com/listings/8e200050-3158-4762-b297-f785b5b1533d"
LEFT_OUT = {"Eye", "Eyelashes", "Teeth_UP", "Teeth_Down"}
"""Under the goggles, or inside the mouth."""
BACK_TO_FRONT = {"Cap", "SKM_Patch_Flag_Cap"}
"""Turned back to front about the cap's own axis: the brim would cut through the goggles."""
COLOURS: dict[str, tuple[str, float]] = {
    # the gear's materials: sRGB colour, roughness
    "goggles": ("#1a1b1f", 0.4),
    "strap": ("#25262b", 0.9),
    "radio": ("#2c2e33", 0.5),
    "sticks": ("#111214", 0.4),
    "antennas": ("#1b1c1f", 0.7),
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
ANTENNA_LENGTH = 22.0
"""Each of the radio's two long-range antennas, standing up from hinges on its top corners."""
ANTENNA_SPLAY_DEG = 12.0
"""Each leans out from upright by this much, in a V."""
CAP_TILT_DEG = 24.0
"""The cap's band leans back this much: its strap sits 8 cm below the brim's seam, 18 cm behind.
Turned about the vertical, the strap would come down over the eyes; about the band's own axis,
it stays up on the forehead, above the goggles, and the brim drops over the nape."""
CAP_PIVOT = np.array([0.0, 175.0, 1.0])
"""Where that axis passes, mid-head: the turned cap still covers the scalp."""
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


def reach(pose: Pose, side: str, wrist: np.ndarray, pole: np.ndarray) -> None:
    """Two-bone IK: the upper arm and forearm turn so the wrist comes to `wrist` (or as near as
    the arm reaches), the elbow bent toward `pole`."""
    shoulder, elbow, hand = (pose.at(f"{b}_{side}") for b in ("upperarm", "lowerarm", "hand"))
    upper, lower = np.linalg.norm(elbow - shoulder), np.linalg.norm(hand - elbow)
    span = wrist - shoulder
    d = min(np.linalg.norm(span), upper + lower - 1e-3)
    toward = span / np.linalg.norm(span)
    pole = unit((pole - (pole @ toward) * toward)[None])[0]
    bend = np.arccos(np.clip((upper**2 + d**2 - lower**2) / (2 * upper * d), -1, 1))
    new_elbow = shoulder + upper * (np.cos(bend) * toward + np.sin(bend) * pole)
    pose.rotate(f"upperarm_{side}", align(elbow - shoulder, new_elbow - shoulder))
    elbow, hand = pose.at(f"lowerarm_{side}"), pose.at(f"hand_{side}")
    pose.rotate(f"lowerarm_{side}", align(hand - elbow, wrist - elbow))


def turn_hand(pose: Pose, side: str, target: np.ndarray) -> None:
    """Turn the hand to `target` (hand_frame's columns), sharing the roll about the forearm out
    along its twist bones so the wrist does not wring."""
    rotation = target @ hand_frame(pose, side).T
    forearm = pose.at(f"hand_{side}") - pose.at(f"lowerarm_{side}")
    roll = twist_about(rotation, forearm)
    pose.rotate(f"hand_{side}", rotation)
    for bone, share in (
        (f"lowerarm_twist_01_{side}", 2 / 3),
        (f"lowerarm_twist_02_{side}", 1 / 3),
    ):
        pose.rotate(bone, axis_angle(forearm, roll * share))


def hold_radio(pose: Pose) -> None:
    axes = radio_axes()
    up, face = axes[:, 1], axes[:, 2]
    for side, out in (("l", 1.0), ("r", -1.0)):
        sideways = np.array([out, 0.0, 0.0])
        wrist = RADIO_CENTRE + WRIST_OUT * sideways + 1.5 * face - 4.0 * up
        reach(pose, side, wrist, ELBOW_POLE * np.array([out, 1.0, 1.0]))

        # The hand: its heel at the radio's lower corner, palm against the side, fingers up
        # the side and round the back, so the thumb (toward the face) comes over the sticks.
        along = unit((up - 0.6 * face)[None])[0]
        palm = -sideways
        turn_hand(pose, side, np.column_stack([along, palm, np.cross(along, palm)]))

        frame = hand_frame(pose, side)
        curl_axis = np.cross(frame[:, 0], frame[:, 1])
        for finger in ("index", "middle", "ring", "pinky"):
            for joint, degrees in zip(("01", "02", "03"), FINGER_CURL_DEG, strict=True):
                pose.rotate(f"{finger}_{joint}_{side}", axis_angle(curl_axis, np.radians(degrees)))


def cap_axis() -> np.ndarray:
    """The cap's own up, square to its band: leaning back by CAP_TILT_DEG."""
    tilt = np.radians(CAP_TILT_DEG)
    return np.array([0.0, np.cos(tilt), -np.sin(tilt)])


def turn_back_to_front(mesh: SkinnedMesh) -> None:
    """Half a turn about the cap's axis through CAP_PIVOT, in place."""
    turn = axis_angle(cap_axis(), np.pi)
    mesh.points[:] = (mesh.points - CAP_PIVOT) @ turn.T + CAP_PIVOT
    mesh.normals[:] = mesh.normals @ turn.T


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


def indexed(
    points: np.ndarray, triangles: np.ndarray, normals: np.ndarray, uvs: np.ndarray, name: str
) -> Primitive:
    """Triangles whose corners share a point, a normal and a UV share a vertex. glTF's v runs
    down the image, FBX's up."""
    corners = triangles.reshape(-1)
    normals, uvs = normals.reshape(-1, 3), uvs.reshape(-1, 2)
    key = np.column_stack([corners, np.round(normals, 3), np.round(uvs, 5)])
    _, first, inverse = np.unique(key, axis=0, return_index=True, return_inverse=True)
    return Primitive(
        name,
        {
            "POSITION": points[corners[first]].astype(np.float32),
            "NORMAL": normals[first].astype(np.float32),
            "TEXCOORD_0": (uvs[first] * [1.0, -1.0] + [0.0, 1.0]).astype(np.float32),
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


def rod(start: np.ndarray, end: np.ndarray, radius: float, name: str, sides: int = 10) -> Primitive:
    """A round rod from `start` to `end`, smooth sided, its far end capped."""
    axis = unit((end - start)[None])[0]
    u = unit(np.cross(axis, [1.0, 0.0, 0.0] if abs(axis[0]) < 0.9 else [0.0, 1.0, 0.0])[None])[0]
    v = np.cross(axis, u)
    positions, normals, indices = [], [], []
    for i in range(sides):
        a = 2 * np.pi * i / sides
        out = np.cos(a) * u + np.sin(a) * v
        positions.extend([start + radius * out, end + radius * out])
        normals.extend([out, out])
        j, k = 2 * i, 2 * ((i + 1) % sides)
        indices.extend([j, k, k + 1, j, k + 1, j + 1])
    tip = len(positions)
    positions.append(end)
    normals.append(axis)
    for i in range(sides):
        a = 2 * np.pi * i / sides
        positions.append(end + radius * (np.cos(a) * u + np.sin(a) * v))
        normals.append(axis)
        indices.extend([tip, tip + 1 + i, tip + 1 + (i + 1) % sides])
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
        indices.extend([j, k + 1, k, j, j + 1, k + 1])
    return Primitive(
        "strap",
        {"POSITION": np.array(positions, np.float32), "NORMAL": np.array(normals, np.float32)},
        np.array(indices, np.uint32),
        None,
    )


def gear(eyes: np.ndarray, head: np.ndarray) -> list[Primitive]:
    """The goggles, their strap, the radio and its antennas. `eyes`: the eyeballs' points;
    `head`: the posed head's. The goggles sit on the bridge of the nose, in front of the eyes."""
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
    antennas = []
    splay = np.radians(ANTENNA_SPLAY_DEG)
    for side in (-1.0, 1.0):
        hinge = RADIO_CENTRE + axes[:, 0] * side * 6.5 + axes[:, 1] * (RADIO_SIZE[1] / 2 + 1.0)
        antennas.append(box(hinge, axes, np.array([2.2, 2.0, 2.2]), "antennas"))
        # Upright in the world, not along the tilted radio, leaning outward.
        direction = np.cos(splay) * np.array([0.0, 1.0, 0.0]) + np.sin(splay) * side * axes[:, 0]
        start = hinge + direction * 0.5
        antennas.append(rod(start, start + direction * ANTENNA_LENGTH, 0.6, "antennas"))
    return [goggles, band, radio, *sticks, *antennas]


def merge(parts: list[Primitive], name: str) -> Primitive:
    offsets = np.cumsum([0] + [len(p.attributes["POSITION"]) for p in parts[:-1]])
    return Primitive(
        name,
        {k: np.concatenate([p.attributes[k] for p in parts]) for k in parts[0].attributes},
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

    groups: dict[str, list[Primitive]] = {}
    head: np.ndarray | None = None
    for mesh in meshes:
        if mesh.name in BACK_TO_FRONT:
            turn_back_to_front(mesh)
        points, normals = bake(mesh, skinning)
        if mesh.name == "Head":
            head = points
        materials = np.array(mesh.materials)
        for material in np.unique(materials):
            if material not in TEXTURED:
                raise FbxFormatError(f"{mesh.name}: no textures for {material}")
            keep = materials == material
            part = indexed(points, mesh.triangles[keep], normals[keep], mesh.uvs[keep], material)
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


def build_glb(source: bytes, textures: Path) -> bytes:
    """The operator's GLB, with the maps in `textures` (tools.models.operatortextures)."""
    gltf = GltfWriter()
    images: list[dict[str, object]] = []

    def texture(material: str, kind: str) -> dict[str, int]:
        data = (textures / f"{material}_{kind}.jpg").read_bytes()
        images.append({"bufferView": gltf.view(data, None), "mimeType": "image/jpeg"})
        return {"index": len(images) - 1}

    children = []
    for part in operator_parts(source):
        if part.name in TEXTURED:
            orm = texture(part.name, "orm")
            material: dict[str, object] = {
                "pbrMetallicRoughness": {
                    "baseColorTexture": texture(part.name, "base"),
                    "metallicRoughnessTexture": orm,
                },
                "normalTexture": texture(part.name, "normal"),
                "occlusionTexture": orm,
            }
        else:
            colour, roughness = COLOURS[part.name]
            material = {
                "pbrMetallicRoughness": {
                    "baseColorFactor": [*linear_rgb(colour), 1.0],
                    "metallicFactor": 0.0,
                    "roughnessFactor": roughness,
                },
            }
        gltf.materials.append({"name": part.name, **material})
        part.material = len(gltf.materials) - 1
        children.append(gltf.mesh_node(part))
    root = len(gltf.nodes)
    gltf.nodes.append({"name": "operator", "children": children})
    asset = {
        "generator": "tools.models.operatormodel",
        "copyright": f'"Quantum Modular Character Free Sample", Fab Standard License, {SOURCE_URL}',
    }
    extra = {
        "images": images,
        "textures": [{"source": i, "sampler": 0} for i in range(len(images))],
        "samplers": [{"magFilter": 9729, "minFilter": 9987}],  # linear, trilinear
    }
    return gltf.to_glb(root, asset, **extra)


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: python -m tools.models.operatormodel <SKM_Character.fbx> <out.glb>"
        )
    out = Path(sys.argv[2])
    out.parent.mkdir(parents=True, exist_ok=True)
    source = Path(sys.argv[1])
    out.write_bytes(build_glb(source.read_bytes(), source.parent / "quantum_textures"))
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
