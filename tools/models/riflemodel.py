"""The ground shooter's rifle in first person: Fab's "Quantum Modular Character Free Sample"
(Standard License) short-barrelled AR with a holographic sight, held in the character's arms.

assets/sm_rifle.fbx is the listing's SM_Rifle.fbx as it came (fbx_all_assets.zip): one static
mesh, in centimetres with +Y up and the muzzle toward +Z, scaled by its model's 0.97. The arms
are the operator's character's, assets/quantum_character.fbx (tools.models.operatormodel): its
bare forearms and hands are the listing's SKM_Arms.fbx point for point, but that file binds
only the hands and forearms' helper bones, while the character has the whole skeleton and the
rolled-up sleeves. After changing this converter, rebuild the model the page serves with

    uv run python -m tools.models.riflemodel assets/sm_rifle.fbx \\
        assets/quantum_character.fbx web/public/models/rifle.glb

It reads the textures from assets/rifle_textures/, beside the sources
(tools.models.rifletextures).

The frame is the shooter's: metres, +Z along the bore toward the muzzle, +Y up, and so (glTF
being right-handed) +X the shooter's left; the origin is the centre of the butt pad, the stock
extended. The rifle splits into connected pieces: the magazine (the lowest piece) and the
holographic sight (the tallest, with the pieces inside its bounds) come out as nodes of their
own, and the sight's glass as a clear child of the sight. The scene's root, "shooter", holds
rifle, holo (holo_glass under it), magazine (its origin the magazine's seat in the well, and
arm_left_mag under it, so the hand goes with the magazine), arm_right and arm_left_grip.

The character's skeleton is turned into a bladed stance, its right shoulder behind the butt,
and each arm reaches its hold by two-bone IK, the wrist's roll shared out along the forearm's
twist bones (operatormodel's reach and turn_hand). The right hand wraps the pistol grip, its
index finger laid straight along the receiver above the trigger guard, off the trigger, and its
thumb round the back of the grip; the left holds the handguard from below, thumb forward along
its left side, or, in the second pose for the reload, the magazine's body. The palm stands on
the rifle's surface and the fingers close until they touch it. The skin is baked in each pose
into a static mesh: the forearm and hand, and the shirt's sleeve up to the shoulder.

The scene's extras are the points the page aims and animates by, measured from the mesh; those
the mesh cannot give (the eye, a rear sight's aperture, the ejection port, the chamber) are
estimates (EST), named under "estimated".
"""

import sys
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial import KDTree

from tools.models.fbx import Bone, FbxFormatError, Scene, SkinnedMesh
from tools.models.fpvmodel import pieces, subset
from tools.models.gltf import GltfWriter, Primitive, unit
from tools.models.operatormodel import (
    Pose,
    align,
    axis_angle,
    bake,
    hand_frame,
    indexed,
    linear_rgb,
    reach,
    turn_hand,
)
from tools.models.rifletextures import SOURCES as TEXTURED

SOURCE_URL = "https://www.fab.com/listings/8e200050-3158-4762-b297-f785b5b1533d"
CM = 0.01

# Where the shooter is, about the rifle. The mesh fixes the rifle's own points; these are not in
# it (EST, from the usual cheek weld behind a holographic sight on a carbine).
EYE_RELIEF = 0.12
"""The eye behind the sight's hood, on its axis."""
RIGHT_SHOULDER = np.array([-0.065, 0.025, -0.05])
"""The right shoulder's joint: just outside and behind the pocket the butt sits in."""
TORSO_YAW_DEG = 40.0
"""The bladed stance: the shoulders' line turned this far from square to the bore, the left
shoulder forward."""
ELBOW_POLES = {
    "r": np.array([-1.0, -0.8, -0.3]),
    "l_grip": np.array([0.4, -1.0, -0.1]),
    "l_mag": np.array([0.6, -1.0, -0.4]),
}
"""Where each elbow points."""
PORT_BEHIND_CHAMBER = 0.045
"""The ejection port's centre behind the chamber (EST: an AR's port is some 5 cm long, its front
edge a centimetre and a half behind the barrel extension). The mesh has neither: its receiver is
closed, and the chamber is taken to be at the upper receiver's front face (EST)."""
GRIP_PALM_ABOVE_MIDDLE = 0.022
"""The middle of the right palm on the grip's side, above the grip's middle."""
INDEX_BELOW_BORE = 0.045
"""The straight index finger lies along the lower receiver this far below the bore, above the
trigger guard, its tip over the magazine well."""
INDEX_CLEAR = 0.008
"""The index finger's bone from the receiver's side: about the finger's half thickness."""
HANDGUARD_HOLD = 0.55
"""Where the left hand holds the handguard, as a fraction of it from the receiver."""
HANDGUARD_CUP = 0.015
"""How far the handguard presses into the flat palm's middle: a real palm cups round it."""
HANDGUARD_PALM_DEG = 20.0
"""The left palm's middle on the handguard: round from straight below toward the left."""
MAG_PALM_BELOW_SEAT = 0.085
"""The middle of the left palm on the magazine's left side, below its seat."""

GLASS = ("#9fc8c0", 0.12)
"""The sight's glass: a faint tint and its opacity. The listing paints a reticle on it, which a
holographic sight shows at infinity, not on the glass; the page draws its own."""
FINGERS = ("index", "middle", "ring", "pinky")
CURL_STEP_DEG = 2.0
FINGER_LIMIT_DEG = (90.0, 100.0, 75.0)
"""Each finger joint's furthest curl, knuckle first."""
FINGER_RATE = (1.0, 1.0, 0.7)
"""How fast each joint closes, against the knuckle."""
THUMB_TIP_LIMIT_DEG = 40.0
"""The thumb's last joint's furthest curl onto the rifle."""
CONTACT = 0.002
"""How far skin may sink into the rifle before a joint stops closing."""
PALM_SINK = 0.004
"""How far the palm's flesh may press into the rifle."""


def surface(
    points: np.ndarray, triangles: np.ndarray, normals: np.ndarray, spacing: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Points spread over the triangles about `spacing` apart, with their triangles' normals
    (the corners' mean: the way the source faces) and indices."""
    t = points[triangles]
    edge = np.max(np.linalg.norm(t - np.roll(t, 1, axis=1), axis=2), axis=1)
    count = np.clip(np.ceil(edge / spacing).astype(int), 1, 64)
    face = unit(normals.mean(axis=1))
    out, out_normals, out_index = [], [], []
    for k in np.unique(count):
        i, j = np.meshgrid(np.arange(k + 1), np.arange(k + 1), indexing="ij")
        keep = i + j <= k
        a, b = (i[keep] / k)[None, :, None], (j[keep] / k)[None, :, None]
        which = np.flatnonzero(count == k)
        tri = t[which]
        spread = tri[:, None, 0] + a * (tri[:, None, 1] - tri[:, None, 0])
        spread = spread + b * (tri[:, None, 2] - tri[:, None, 0])
        out.append(spread.reshape(-1, 3))
        out_normals.append(np.repeat(face[which], a.shape[1], axis=0))
        out_index.append(np.repeat(which, a.shape[1]))
    return np.vstack(out), np.vstack(out_normals), np.concatenate(out_index)


class Solid:
    """The rifle's surface, to tell skin that has sunk into it."""

    def __init__(self, points: np.ndarray, normals: np.ndarray) -> None:
        self.points, self.normals = points, normals
        self.tree = KDTree(points)

    def depth(self, query: np.ndarray) -> np.ndarray:
        """How far each point is under the surface, by the nearest surface point's side
        (negative: outside it; far from any surface, -inf)."""
        distance, nearest = self.tree.query(query, distance_upper_bound=0.02)
        nearest = np.minimum(nearest, len(self.points) - 1)
        signed = np.einsum("ij,ij->i", query - self.points[nearest], self.normals[nearest])
        return np.where(distance < 0.02, -signed, -np.inf)


@dataclass
class Rifle:
    """The rifle's parts in the shooter's frame, its surface, and its measured points."""

    rifle: Primitive
    magazine: Primitive
    holo: Primitive
    glass: Primitive
    solid: Solid
    points: dict[str, np.ndarray]


def _slices(points: np.ndarray, axis: int, levels: Iterable[float], half: float) -> np.ndarray:
    """The bounding-box centre of `points` in each slab `half` either side of each level."""
    out = []
    for level in levels:
        slab = points[np.abs(points[:, axis] - level) < half]
        out.append((slab.min(axis=0) + slab.max(axis=0)) / 2)
    return np.array(out)


def _fit_direction(centres: np.ndarray) -> np.ndarray:
    """The unit direction of the line through `centres`, from the first toward the last."""
    direction = np.linalg.svd(centres - centres.mean(axis=0))[2][0]
    return direction if direction @ (centres[-1] - centres[0]) > 0 else -direction


def rifle_parts(source: bytes) -> Rifle:
    """The rifle split into its nodes' parts, in the shooter's frame, and its points."""
    (mesh,) = Scene(source).meshes()
    whole = indexed(mesh.points, mesh.triangles, mesh.normals, mesh.uvs, "rifle")
    labels = pieces(whole)  # in the source's centimetres: pieces a hair apart stay apart
    triangles = whole.indices.reshape(-1, 3)
    every = whole.attributes["POSITION"].astype(np.float64) * CM

    ids = [int(k) for k in np.unique(labels)]
    bounds = {}
    for k in ids:
        points = every[np.unique(triangles[labels == k])]
        bounds[k] = (points.min(axis=0), points.max(axis=0))
    magazine = min(ids, key=lambda k: bounds[k][0][1])
    hood = max(ids, key=lambda k: bounds[k][1][1])
    holo = [
        k
        for k in ids
        if np.all(bounds[hood][0] <= sum(bounds[k]) / 2)
        and np.all(sum(bounds[k]) / 2 <= bounds[hood][1])
    ]
    flat = [k for k in holo if bounds[k][1][2] - bounds[k][0][2] < 1e-4]
    if len(flat) != 1:
        raise FbxFormatError(f"the sight should have one flat glass, not {len(flat)}")
    glass = flat[0]

    # The origin: the butt pad's centre, at the rearmost piece's back.
    stock = min(ids, key=lambda k: bounds[k][0][2])
    butt_z = bounds[stock][0][2]
    pad = every[np.abs(every[:, 2] - butt_z) < 0.005]
    butt = np.array([*(pad[:, :2].min(axis=0) + pad[:, :2].max(axis=0)) / 2, butt_z])
    every = every - butt

    def part(keep: np.ndarray, name: str) -> Primitive:
        out = subset(whole, keep, name)
        out.attributes["POSITION"] = (
            out.attributes["POSITION"].astype(np.float64) * CM - butt
        ).astype(np.float32)
        return out

    # Points spread 2 mm apart over the surface, to measure by and to keep the hands out.
    normals = whole.attributes["NORMAL"].astype(np.float64)[triangles]
    samples, sample_normals, sample_triangle = surface(every, triangles, normals, 0.002)
    sample_piece = labels[sample_triangle]

    def of(*keep: int) -> np.ndarray:
        return samples[np.isin(sample_piece, keep)]

    barrel = max(ids, key=lambda k: bounds[k][1][2])
    points = measure(
        of(stock),
        of(barrel),
        of(*[k for k in holo if k != glass]),
        every[np.unique(triangles[labels == glass])],
        of(magazine),
        samples[~np.isin(sample_piece, holo)],
    )
    in_holo = np.isin(labels, holo)
    solid = sample_piece != glass
    return Rifle(
        part(~in_holo & (labels != magazine), "rifle"),
        part(labels == magazine, "magazine"),
        part(in_holo & (labels != glass), "holo"),
        part(labels == glass, "holo_glass"),
        Solid(samples[solid], sample_normals[solid]),
        points,
    )


def measure(
    body: np.ndarray,
    barrel: np.ndarray,
    sight: np.ndarray,
    glass: np.ndarray,
    magazine: np.ndarray,
    rest: np.ndarray,
) -> dict[str, np.ndarray]:
    """The rifle's points of interest, from points spread over its surface: the receiver and
    stock (`body`), the barrel, the sight without its `glass`, the magazine and everything but
    the sight (`rest`)."""
    p: dict[str, np.ndarray] = {"butt": np.zeros(3)}

    # The bore: the barrel's axis, out to the muzzle device's face.
    bore = (barrel[:, :2].min(axis=0) + barrel[:, :2].max(axis=0)) / 2
    front_face = body[:, 2].max()
    chamber = np.array([*bore, front_face])
    p["muzzle"] = np.array([*bore, barrel[:, 2].max()])
    p["bore"] = np.array([chamber, p["muzzle"]])

    # The sight: its glass's corners (left is +X), and its axis through the hood's openings.
    (x0, y0, _), (x1, y1, _) = glass.min(axis=0), glass.max(axis=0)
    z = float(glass[:, 2].mean())
    p["holo_window"] = np.array([[x1, y1, z], [x0, y1, z], [x0, y0, z], [x1, y0, z]])
    hood = sight[sight[:, 1] > y1 + 0.001]
    middle = np.array([(x0 + x1) / 2, (y0 + y1) / 2])
    p["holo_axis"] = np.array([[*middle, hood[:, 2].min()], [*middle, hood[:, 2].max()]])
    p["eye"] = np.array([*middle, hood[:, 2].min() - EYE_RELIEF])

    # The front sight: the post between its ears, on the handguard's rail.
    rail_top = body[:, 1].max()
    ahead = rest[(rest[:, 2] > front_face + 0.005) & (rest[:, 1] > rail_top - 0.006)]
    post = ahead[np.abs(ahead[:, 0] - bore[0]) < 0.0015]
    tip = post[post[:, 1] > post[:, 1].max() - 0.0005]
    p["front_post_tip"] = np.array([bore[0], post[:, 1].max(), tip[:, 2].mean()])
    # No rear sight in the mesh: a flip-up's aperture on the rail between its end and the
    # holographic sight, as high over the bore as the post's tip for a co-witness (EST).
    rail = body[body[:, 1] > rail_top - 0.0005]
    p["rear_aperture"] = np.array(
        [bore[0], p["front_post_tip"][1], (rail[:, 2].min() + sight[:, 2].min()) / 2]
    )

    # The ejection port: on the right (-X) of the upper receiver, at the bore's height (EST).
    port_z = chamber[2] - PORT_BEHIND_CHAMBER
    wall = body[(np.abs(body[:, 2] - port_z) < 0.003) & (np.abs(body[:, 1] - bore[1]) < 0.003)]
    p["ejection_port"] = np.array([wall[:, 0].min(), bore[1], port_z])
    p["ejection_dir"] = np.array([-1.0, 0.0, 0.0])

    # The magazine: its top seats in the well; its straight upper part is the well's axis.
    top = magazine[:, 1].max()
    centres = _slices(magazine, 1, top - np.linspace(0.01, 0.035, 6), 0.002)
    p["mag_axis"] = _fit_direction(centres)
    p["mag_well"] = centres[0] + (top - centres[0][1]) / p["mag_axis"][1] * p["mag_axis"]

    # The pistol grip: the stock and receiver's piece below the trigger guard, behind the
    # magazine. Its lower edge, along the bore: the grip's bottom, then the guard.
    lower = body[body[:, 2] < magazine[:, 2].min()]
    bins = np.round(lower[:, 2] / 0.005).astype(int)
    floor = {int(b): float(lower[bins == b][:, 1].min()) for b in np.unique(bins)}
    bottom = min(floor.values())
    grip_bins = [b for b, y in floor.items() if y < bottom + 0.02]
    guard = min(y for b, y in floor.items() if b > max(grip_bins) + 6)
    grip = lower[(lower[:, 1] < guard - 0.003) & (lower[:, 2] > min(grip_bins) * 0.005 - 0.02)]
    centres = _slices(grip, 1, np.linspace(guard - 0.01, bottom + 0.01, 8), 0.002)
    p["grip_axis"] = _fit_direction(centres)
    p["grip"] = _slices(grip, 1, [(guard + bottom) / 2], 0.002)[0]
    p["grip_top"] = _slices(grip, 1, [guard - 0.006], 0.002)[0]

    # The handguard's front: the furthest of what stands above the barrel.
    above = rest[(rest[:, 2] > front_face) & (rest[:, 1] > barrel[:, 1].max() + 0.005)]
    p["handguard_front"] = np.array([*bore, above[:, 2].max()])
    return p


def _metres(bone: Bone) -> Bone:
    bind = bone.bind.copy()
    bind[:3, 3] *= CM
    return Bone(bone.name, bone.parent, bind)


class Skin:
    """The arms' skin (or sleeves'), to pose a few of its points at a time."""

    def __init__(self, mesh: SkinnedMesh, pose: Pose) -> None:
        self.mesh = mesh
        self.names = sorted(mesh.weights)
        self.weights = np.zeros((len(mesh.points), len(self.names)))
        for b, name in enumerate(self.names):
            indices, values = mesh.weights[name]
            self.weights[indices, b] = values
        self.dominant = [self.names[b] for b in np.argmax(self.weights, axis=1)]
        self.pose = pose

    def points(self, indices: np.ndarray) -> np.ndarray:
        """Points `indices` in the pose as it stands."""
        skinning = self.pose.skinning()
        m = np.array([skinning[n] for n in self.names])
        w = self.weights[indices]
        moved = np.einsum("bij,kj->kbi", m[:, :3, :3], self.mesh.points[indices]) + m[:, :3, 3]
        return np.einsum("kb,kbi->ki", w, moved)

    def palm(self, side: str) -> np.ndarray:
        """The points the hand's own bones and the fingers' metacarpals carry."""
        bones = (f"hand_{side}",) + tuple(f"{f}_metacarpal_{side}" for f in FINGERS)
        return np.flatnonzero(np.isin(self.dominant, bones))

    def under(self, joints: list[str]) -> list[np.ndarray]:
        """For each joint of a chain, the points its bone carries and the next joint's does
        not: each point goes to the nearest of the joints above its strongest bone."""
        out: list[list[int]] = [[] for _ in joints]
        for i, name in enumerate(self.dominant):
            while name is not None and name not in joints:
                name = self.pose.bones[name].parent
            if name is not None:
                out[joints.index(name)].append(i)
        return [np.array(o, dtype=np.int64) for o in out]


def stance(bones: dict[str, Bone]) -> Pose:
    """The skeleton turned into the bladed stance, its right shoulder at RIGHT_SHOULDER."""
    pose = Pose(bones)
    yaw = np.radians(-TORSO_YAW_DEG)
    move = np.eye(4)
    move[:3, :3] = turn = axis_angle(np.array([0.0, 1.0, 0.0]), yaw)
    move[:3, 3] = RIGHT_SHOULDER - turn @ pose.at("upperarm_r")
    for name in pose.posed:
        pose.posed[name] = move @ pose.posed[name]
    return pose


def palm_offset(pose: Pose, skin: Skin, side: str) -> np.ndarray:
    """The middle of the palm's skin, under the middle finger's bones, in hand_frame's axes
    from the wrist."""
    frame = hand_frame(pose, side)
    wrist, knuckle = pose.at(f"hand_{side}"), pose.at(f"middle_01_{side}")
    hand = np.flatnonzero(np.array([n.endswith(f"_{side}") for n in skin.dominant]))
    local = (skin.points(hand) - wrist) @ frame
    middle = frame.T @ ((wrist + knuckle) / 2 - wrist)
    near = np.linalg.norm((local - middle)[:, [0, 2]], axis=1) < 0.012
    return local[near][np.argmax(local[near][:, 1])]


def close(
    pose: Pose,
    skin: Skin,
    solid: Solid,
    joints: list[str],
    palm: np.ndarray,
    limits: tuple[float, ...],
    rates: tuple[float, ...],
) -> None:
    """Curl a finger's joints toward the palm together until each touches the rifle: when a
    bone would sink into it, that joint and those nearer the palm stop (and, if they had
    stopped already, the nearest of those still closing)."""
    direction = pose.at(joints[-1]) - pose.at(joints[0])
    if len(joints) == 1:
        direction = pose.at(joints[0]) - pose.at(str(pose.bones[joints[0]].parent))
    axis = unit(np.cross(direction, palm)[None])[0]
    links = skin.under(joints)
    every = np.concatenate(links)
    owner = np.concatenate([np.full(len(link), i) for i, link in enumerate(links)])
    allowed = np.maximum(CONTACT, solid.depth(skin.points(every)))  # some start in contact
    angles = np.zeros(len(joints))
    active = [True] * len(joints)
    while True:
        saved = {b: pose.posed[b].copy() for b in pose.posed}
        moved = []
        for i, joint in enumerate(joints):
            step = CURL_STEP_DEG * rates[i]
            if active[i] and angles[i] + step <= limits[i]:
                pose.rotate(joint, axis_angle(axis, np.radians(step)))
                angles[i] += step
                moved.append(i)
        if not moved:
            return
        sunk = solid.depth(skin.points(every)) > allowed
        if sunk.any():
            pose.posed = saved
            for i in moved:
                angles[i] -= CURL_STEP_DEG * rates[i]
            deepest = int(owner[sunk].max())
            for i in range(max(deepest, min(moved)) + 1):
                active[i] = False


def extent(solid: Solid, centre: np.ndarray, half: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The bounds of the rifle's surface within a box."""
    inside = np.all(np.abs(solid.points - centre) <= half, axis=1)
    return solid.points[inside].min(axis=0), solid.points[inside].max(axis=0)


def first_hit(solid: Solid, start: np.ndarray, direction: np.ndarray) -> np.ndarray:
    """Where a ray from `start` first meets the rifle's surface (within 2 mm of it)."""
    direction = direction / np.linalg.norm(direction)
    offset = solid.points - start
    along = offset @ direction
    off_ray = np.linalg.norm(offset - along[:, None] * direction, axis=1)
    near = (off_ray < 0.002) & (along > 0)
    if not near.any():
        raise FbxFormatError(f"nothing of the rifle along {direction} from {start}")
    return start + along[near].min() * direction


def hold(
    pose: Pose,
    skin: Skin,
    solid: Solid,
    side: str,
    contact: np.ndarray,
    along: np.ndarray,
    palm: np.ndarray,
    pole: np.ndarray,
    sink: float = PALM_SINK,
) -> None:
    """Reach the hand to the rifle: the middle of its palm on `contact`, facing `palm`, the
    fingers' bones along `along` before they curl; backed off along the palm's normal as far
    as the palm would otherwise sink into the rifle by more than `sink`."""
    along = unit((along - (along @ palm) * palm)[None])[0]
    target = np.column_stack([along, palm, np.cross(along, palm)])
    offset = palm_offset(pose, skin, side)
    flesh = skin.palm(side)
    for _ in range(6):
        for _ in range(3):  # turning the hand moves the palm a little off the wrist's line
            reach(pose, side, contact - target @ offset, pole)
            turn_hand(pose, side, target)
        sunk = float(solid.depth(skin.points(flesh)).max())
        if sunk <= sink:
            return
        contact = contact - palm * (sunk - sink / 2)


def lay(pose: Pose, skin: Skin, solid: Solid, aims: list[tuple[str, str, np.ndarray]]) -> None:
    """Turn each (joint, bone beyond it, target) in turn so the bone beyond points at the
    target, the targets pushed off the rifle's surface until the bones they move clear it."""
    saved = {b: m.copy() for b, m in pose.posed.items()}
    flesh = np.concatenate(skin.under([joint for joint, _, _ in aims]))
    targets = [t.copy() for _, _, t in aims]
    for _ in range(20):
        pose.posed = {b: m.copy() for b, m in saved.items()}
        for (joint, beyond, _), target in zip(aims, targets, strict=True):
            pose.rotate(joint, align(pose.at(beyond) - pose.at(joint), target - pose.at(joint)))
        points = skin.points(flesh)
        depth = solid.depth(points)
        if depth.max() <= CONTACT:
            return
        _, nearest = solid.tree.query(points[np.argmax(depth)])
        targets = [t + solid.normals[nearest] * 0.002 for t in targets]


def grip_fingers(pose: Pose, skin: Skin, solid: Solid, side: str, fingers: Iterable[str]) -> None:
    palm = hand_frame(pose, side)[:, 1]
    for finger in fingers:
        joints = [f"{finger}_0{j}_{side}" for j in (1, 2, 3)]
        close(pose, skin, solid, joints, palm, FINGER_LIMIT_DEG, FINGER_RATE)


def place_thumb(
    pose: Pose, skin: Skin, solid: Solid, side: str, knuckle: np.ndarray, tip: np.ndarray
) -> None:
    """Lay the thumb with its knuckle (the base of its first phalanx) toward `knuckle` and
    its phalanges toward `tip`, then close its last joint onto the rifle."""
    base, mid, end = (f"thumb_0{j}_{side}" for j in (1, 2, 3))
    lay(pose, skin, solid, [(base, mid, knuckle), (mid, end, tip)])
    palm = hand_frame(pose, side)[:, 1]
    close(pose, skin, solid, [end], palm, (THUMB_TIP_LIMIT_DEG,), (1.0,))


def right_arm(pose: Pose, skin: Skin, rifle: Rifle) -> None:
    """The right hand round the pistol grip, the index finger along the receiver above the
    trigger guard, off the trigger."""
    p = rifle.points
    down = p["grip_axis"]
    forward = np.array([0.0, 0.0, 1.0])
    along = forward - (forward @ down) * down
    at = p["grip"] - down * GRIP_PALM_ABOVE_MIDDLE
    contact = first_hit(rifle.solid, at + np.array([-0.08, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
    hold(pose, skin, rifle.solid, "r", contact, along, np.array([1.0, 0.0, 0.0]), ELBOW_POLES["r"])
    side = first_hit(
        rifle.solid,
        np.array([-0.08, p["bore"][0][1] - INDEX_BELOW_BORE, p["mag_well"][2]]),
        np.array([1.0, 0.0, 0.0]),
    )
    at = side + np.array([-INDEX_CLEAR, 0.0, 0.0])
    lay(pose, skin, rifle.solid, [("index_01_r", "index_03_r", at)])
    grip_fingers(pose, skin, rifle.solid, "r", FINGERS[1:])
    # The thumb round the back of the grip's top, onto its left side.
    top = p["grip_top"]
    low, high = extent(rifle.solid, top, np.array([0.03, 0.002, 0.04]))
    knuckle = np.array([top[0] - 0.010, top[1] + 0.003, low[2] - 0.014])
    tip = np.array([high[0] + 0.010, top[1] + 0.008, low[2] + 0.03])
    place_thumb(pose, skin, rifle.solid, "r", knuckle, tip)


def left_arm_grip(pose: Pose, skin: Skin, rifle: Rifle) -> None:
    """The left hand under the handguard near its front, thumb forward along its left side."""
    p = rifle.points
    front = p["handguard_front"][2]
    z = p["bore"][0][2] + HANDGUARD_HOLD * (front - p["bore"][0][2])
    angle = np.radians(HANDGUARD_PALM_DEG)
    inward = np.array([-np.sin(angle), np.cos(angle), 0.0])
    axis_point = np.array([*p["bore"][0][:2], z])
    contact = first_hit(rifle.solid, axis_point - 0.1 * inward, inward)
    under = -np.cross(inward, [0.0, 0.0, 1.0]) + 0.6 * np.array([0.0, 0.0, 1.0])
    contact = contact + inward * HANDGUARD_CUP
    hold(pose, skin, rifle.solid, "l", contact, under, inward, ELBOW_POLES["l_grip"], 0.02)
    grip_fingers(pose, skin, rifle.solid, "l", FINGERS)
    # The thumb forward along the handguard's left side.
    low, high = extent(rifle.solid, axis_point, np.array([0.04, 0.04, 0.003]))
    middle = (low + high) / 2
    knuckle = np.array([high[0] + 0.014, middle[1] - 0.02, contact[2] + 0.02])
    tip = np.array([high[0] + 0.010, middle[1] - 0.008, contact[2] + 0.09])
    place_thumb(pose, skin, rifle.solid, "l", knuckle, tip)


def left_arm_mag(pose: Pose, skin: Skin, rifle: Rifle) -> None:
    """The left hand round the magazine's body, palm on its left side, fingers round its
    front."""
    p = rifle.points
    at = p["mag_well"] + p["mag_axis"] * MAG_PALM_BELOW_SEAT
    contact = first_hit(rifle.solid, at + np.array([0.08, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0]))
    along = np.array([0.0, -0.3, 1.0])
    palm = np.array([-1.0, 0.0, 0.0])
    hold(pose, skin, rifle.solid, "l", contact, along, palm, ELBOW_POLES["l_mag"])
    grip_fingers(pose, skin, rifle.solid, "l", FINGERS)
    # The thumb round the magazine's back edge.
    low, high = extent(rifle.solid, contact, np.array([0.03, 0.002, 0.06]))
    knuckle = np.array([high[0] + 0.006, contact[1] + 0.012, low[2] - 0.012])
    tip = np.array([low[0], contact[1] + 0.015, low[2] - 0.009])
    place_thumb(pose, skin, rifle.solid, "l", knuckle, tip)


ARMS = "Quantum_Arms"
SLEEVES = "Shirt_RolledUp"
"""The character's meshes the arms are made of: bare forearms and hands, and the shirt."""


def arm_parts(pose: Pose, meshes: dict[str, SkinnedMesh], side: str) -> list[Primitive]:
    """One arm's forearm and hand, and its sleeve up to the shoulder, baked in the pose: the
    triangles whose corners are each bound at least half to that side's arm."""
    skinning = pose.skinning()
    out = []
    for name in (ARMS, SLEEVES):
        mesh = meshes[name]
        share = np.zeros(len(mesh.points))
        for bone, (indices, weights) in mesh.weights.items():
            if bone.endswith(f"_{side}") and not bone.startswith("clavicle"):
                np.add.at(share, indices, weights)
        keep = np.all(share[mesh.triangles] >= 0.5, axis=1)
        (material,) = set(np.array(mesh.materials)[keep])
        points, normals = bake(mesh, skinning)
        out.append(indexed(points, mesh.triangles[keep], normals[keep], mesh.uvs[keep], material))
    return out


@dataclass
class Shooter:
    rifle: Rifle
    arm_right: list[Primitive]
    arm_left_grip: list[Primitive]
    arm_left_mag: list[Primitive]


def shooter(rifle_source: bytes, character_source: bytes) -> Shooter:
    """The rifle and the arms in their three poses, in the shooter's frame."""
    rifle = rifle_parts(rifle_source)
    scene = Scene(character_source)
    bones = {name: _metres(b) for name, b in scene.bones().items()}
    meshes = {m.name: m for m in scene.meshes() if m.name in (ARMS, SLEEVES)}
    for mesh in meshes.values():
        mesh.points *= CM
    arms = []
    for side, pose_arm in (("r", right_arm), ("l", left_arm_grip), ("l", left_arm_mag)):
        pose = stance(bones)
        pose_arm(pose, Skin(meshes[ARMS], pose), rifle)
        arms.append(arm_parts(pose, meshes, side))
    return Shooter(rifle, *arms)


def _rounded(value: np.ndarray) -> object:
    return np.round(value, 5).tolist()


def extras(rifle: Rifle) -> dict[str, object]:
    """The scene's extras: the rifle's points, in metres in the model's frame."""
    p = rifle.points
    out: dict[str, object] = {
        "frame": "metres; +Z along the bore to the muzzle, +Y up, +X the shooter's left; "
        "origin the butt pad's centre, stock extended",
    }
    for key in (
        "eye",
        "holo_window",
        "holo_axis",
        "front_post_tip",
        "rear_aperture",
        "bore",
        "muzzle",
        "ejection_port",
        "ejection_dir",
        "mag_well",
        "mag_axis",
        "butt",
        "grip",
    ):
        out[key] = _rounded(p[key])
    every = np.vstack(
        [part.attributes["POSITION"] for part in (rifle.rifle, rifle.magazine, rifle.holo)]
    )
    out["length"] = round(float(np.ptp(every[:, 2])), 4)
    out["sight_height"] = round(float(p["holo_axis"][0][1] - p["bore"][0][1]), 4)
    out["estimated"] = ["eye", "rear_aperture", "ejection_port", "ejection_dir", "bore[0]"]
    return out


def build_glb(rifle_source: bytes, character_source: bytes, textures: Path) -> bytes:
    """The rifle and arms' GLB, with the maps in `textures` (tools.models.rifletextures)."""
    made = shooter(rifle_source, character_source)
    rifle = made.rifle
    points = extras(rifle)
    gltf = GltfWriter()
    images: list[dict[str, object]] = []
    materials: dict[str, int] = {}

    def texture(material: str, kind: str) -> dict[str, int]:
        data = (textures / f"{material}_{kind}.jpg").read_bytes()
        images.append({"bufferView": gltf.view(data, None), "mimeType": "image/jpeg"})
        return {"index": len(images) - 1}

    for name in TEXTURED:
        orm = texture(name, "orm")
        gltf.materials.append(
            {
                "name": name,
                "pbrMetallicRoughness": {
                    "baseColorTexture": texture(name, "base"),
                    "metallicRoughnessTexture": orm,
                },
                "normalTexture": texture(name, "normal"),
                "occlusionTexture": orm,
            }
        )
        materials[name] = len(gltf.materials) - 1
    colour, opacity = GLASS
    gltf.materials.append(
        {
            "name": "holo_glass",
            "pbrMetallicRoughness": {
                "baseColorFactor": [*linear_rgb(colour), opacity],
                "metallicFactor": 0.0,
                "roughnessFactor": 0.05,
            },
            "alphaMode": "BLEND",
        }
    )
    materials["holo_glass"] = len(gltf.materials) - 1

    def node(name: str, parts: list[Primitive], origin: np.ndarray | None = None) -> int:
        """A node of `parts`, their positions from `origin` (the node's own) if given."""
        for part in parts:
            part.material = materials.get(part.name, materials["Rifle"])
            if origin is not None:
                part.attributes["POSITION"] = part.attributes["POSITION"] - origin.astype(
                    np.float32
                )
        return gltf.parts_node(name, parts)

    seat = rifle.points["mag_well"]
    glass = node("holo_glass", [rifle.glass])
    holo = node("holo", [rifle.holo])
    gltf.nodes[holo]["children"] = [glass]
    hand_on_mag = node("arm_left_mag", made.arm_left_mag, seat)
    magazine = node("magazine", [rifle.magazine], seat)
    gltf.nodes[magazine]["translation"] = _rounded(seat)
    gltf.nodes[magazine]["children"] = [hand_on_mag]
    children = [
        node("rifle", [rifle.rifle]),
        holo,
        magazine,
        node("arm_right", made.arm_right),
        node("arm_left_grip", made.arm_left_grip),
    ]
    root = len(gltf.nodes)
    gltf.nodes.append({"name": "shooter", "children": children})
    asset = {
        "generator": "tools.models.riflemodel",
        "copyright": f'"Quantum Modular Character Free Sample", Fab Standard License, {SOURCE_URL}',
    }
    extra = {
        "images": images,
        "textures": [{"source": i, "sampler": 0} for i in range(len(images))],
        "samplers": [{"magFilter": 9729, "minFilter": 9987}],  # linear, trilinear
    }
    return gltf.to_glb(root, asset, points, **extra)


def main() -> None:
    if len(sys.argv) != 4:
        raise SystemExit(
            "usage: python -m tools.models.riflemodel <SM_Rifle.fbx> <SKM_Character.fbx> <out.glb>"
        )
    out = Path(sys.argv[3])
    out.parent.mkdir(parents=True, exist_ok=True)
    rifle, character = Path(sys.argv[1]), Path(sys.argv[2])
    glb = build_glb(rifle.read_bytes(), character.read_bytes(), rifle.parent / "rifle_textures")
    out.write_bytes(glb)
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
