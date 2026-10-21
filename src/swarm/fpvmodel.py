"""Sketchfab's "FPV Drone" by PinaPipe (CC BY 4.0) as the COP's drone model.

assets/fpv_drone.glb is the Sketchfab download as it came (GLB, 1k textures). Sketchfab needs
a login, so it is committed rather than fetched. After changing this converter, rebuild the
model the COP serves with

    uv run python -m swarm.fpvmodel assets/fpv_drone.glb src/swarm/web/models/fpv-drone.glb

This bakes the node transforms into the vertices and turns the frame so its FPV camera faces
glTF +Z with +Y up, in metres, centred between the motors on the ground. It splits the four
propellers out of the motor mesh into nodes on their hubs, named by corner. The COP spins them
(fleet.js) ArduPilot's quad X way, front right anticlockwise; build_glb checks the blades are
pitched for that, and records each prop's turn in its node's extras. It drops the UV sets no
material reads and scales the textures down to TEXTURE_PX.
"""

import sys
from pathlib import Path

import numpy as np

from swarm.gltf import (
    GlbFormatError,
    GltfWriter,
    Primitive,
    read_glb,
    scene_primitives,
    transform,
    unit,
    yaw,
)

SOURCE_URL = "https://sketchfab.com/3d-models/fpv-drone-92152c31ec9a4a87b2a0478bd114d293"
PROP_MESH = "Motor_Top_Motor_Top_0"
"""The source mesh holding the propellers, with the motor tops, shafts and prop nuts."""
HUB_MESH = "Motor_Rotor_Rotor_0"
"""The source mesh of the motor bells: a propeller's hub is the centre of the bell under it."""
CAMERA_XZ = (0.36, 1.1)
"""Where the FPV camera sits in the source scene (x, z), found by inspection. It only picks
which side of the frame is the front; the frame's own symmetry sets the exact heading."""
PROP_DIAMETER_M = 5 * 0.0254
"""Sets the scale: the source has no units, and the propellers are 5 inch."""
TEXTURE_PX = 512
"""Largest texture side. The drone is a few hundred pixels across at most, even close up."""
ARDUPILOT_QUAD_X = {
    "prop_front_right": 1,
    "prop_rear_left": 1,
    "prop_front_left": -1,
    "prop_rear_right": -1,
}
"""Each corner's turn seen from above, +1 anticlockwise: motors 1 and 2 anticlockwise, 3 and 4
clockwise."""


def pieces(primitive: Primitive) -> np.ndarray:
    """Per triangle, a label shared by the triangles of each connected piece. Vertices at the
    same position count as joined, so UV seams do not split a piece."""
    _, welded = np.unique(
        np.round(primitive.attributes["POSITION"], 4), axis=0, return_inverse=True
    )
    triangles = welded.reshape(-1)[primitive.indices.reshape(-1, 3)]
    labels = np.arange(int(triangles.max()) + 1)
    while True:
        lowest = labels[triangles].min(axis=1)
        merged = labels.copy()
        np.minimum.at(merged, triangles.reshape(-1), np.repeat(lowest, 3))
        merged = merged[merged]
        if np.array_equal(merged, labels):
            return labels[triangles[:, 0]]
        labels = merged


def subset(primitive: Primitive, keep: np.ndarray, name: str) -> Primitive:
    """The triangles where `keep` (per triangle) is set, with only the vertices they use."""
    triangles = primitive.indices.reshape(-1, 3)[keep]
    used, remapped = np.unique(triangles, return_inverse=True)
    return Primitive(
        name,
        {k: v[used] for k, v in primitive.attributes.items()},
        remapped.reshape(-1).astype(np.uint32),
        primitive.material,
    )


def _bounds_centre(points: np.ndarray) -> np.ndarray:
    return (points.min(axis=0) + points.max(axis=0)) / 2.0


def split_props(motors: Primitive) -> tuple[Primitive, list[Primitive]]:
    """The motor mesh without its four propellers, and the propellers: the four pieces that
    reach furthest across."""
    labels = pieces(motors)
    positions = motors.attributes["POSITION"]
    triangles = motors.indices.reshape(-1, 3)
    span: dict[int, float] = {}
    for label in np.unique(labels):
        flat = positions[np.unique(triangles[labels == label])][:, [0, 2]]
        span[int(label)] = float(np.linalg.norm(flat.max(axis=0) - flat.min(axis=0)))
    ranked = sorted(span, key=span.__getitem__, reverse=True)
    if len(ranked) < 5 or span[ranked[3]] < 2.0 * span[ranked[4]]:
        raise GlbFormatError(f"{motors.name}: no four propellers stand out")
    props = [subset(motors, labels == label, "prop") for label in ranked[:4]]
    return subset(motors, ~np.isin(labels, ranked[:4]), motors.name), props


def hubs(props: list[Primitive], bells: Primitive) -> np.ndarray:
    """(4, 3): each propeller's hub, the centre of the motor bell nearest it, at the
    propeller's mid-height."""
    labels = pieces(bells)
    positions = bells.attributes["POSITION"]
    triangles = bells.indices.reshape(-1, 3)
    centres = np.array(
        [_bounds_centre(positions[np.unique(triangles[labels == k])]) for k in np.unique(labels)]
    )
    out = []
    for prop in props:
        points = prop.attributes["POSITION"]
        mid = points.mean(axis=0)
        bell = centres[np.argmin(np.linalg.norm((centres - mid)[:, [0, 2]], axis=1))]
        out.append([bell[0], _bounds_centre(points)[1], bell[2]])
    return np.array(out)


def nose_rotation(hub_points: np.ndarray, camera_xz: tuple[float, float]) -> np.ndarray:
    """Rotation about +Y putting the frame's front on +Z: of the four directions from the
    centre between the hubs to the middle of two neighbouring hubs, the one nearest the
    camera."""
    flat = hub_points[:, [0, 2]]
    centre = flat.mean(axis=0)
    order = np.argsort(np.arctan2(*(flat - centre).T[::-1]))
    ring = flat[order]
    sides = (ring + np.roll(ring, -1, axis=0)) / 2.0 - centre
    camera = np.array(camera_xz) - centre
    front = sides[np.argmax(unit(sides) @ camera)]
    return yaw(front)


def spin_direction(points: np.ndarray, indices: np.ndarray) -> int:
    """+1 (anticlockwise seen from above, right-handed about +Y) or -1: the way a propeller
    must turn to push air down. `points` are relative to its hub. A blade's leading edge sits
    higher than its trailing edge, so its faces tilt back against the way it turns; summed
    over face area, with top and bottom faces counted alike, so winding does not matter."""
    a, b, c = (points[indices.reshape(-1, 3)[:, k]].astype(np.float64) for k in range(3))
    face = np.cross(b - a, c - a)
    centre = (a + b + c) / 3.0
    r = np.maximum(np.linalg.norm(centre[:, [0, 2]], axis=1), 1e-9)
    ahead = np.c_[centre[:, 2] / r, np.zeros_like(r), -centre[:, 0] / r]  # +Y cross radius
    tilt = np.sign(face[:, 1]) * np.sum(face * ahead, axis=1)
    return 1 if float(tilt.sum()) < 0.0 else -1


def _prop_name(hub: np.ndarray) -> str:
    """glTF +X is the drone's left."""
    return f"prop_{'front' if hub[2] > 0 else 'rear'}_{'left' if hub[0] > 0 else 'right'}"


def build_glb(source: bytes, texture_px: int | None = TEXTURE_PX) -> bytes:
    doc, buffer = read_glb(source)
    parts = {p.name: p for p in scene_primitives(doc, buffer)}
    static, props = split_props(parts[PROP_MESH])
    parts[PROP_MESH] = static
    hub_points = hubs(props, parts[HUB_MESH])
    rotation = nose_rotation(hub_points, CAMERA_XZ)
    diameter = 2.0 * max(
        float(np.linalg.norm((p.attributes["POSITION"] - h)[:, [0, 2]], axis=1).max())
        for p, h in zip(props, hub_points, strict=True)
    )
    scale = PROP_DIAMETER_M / diameter
    lowest = min(float((p.attributes["POSITION"] @ rotation.T)[:, 1].min()) for p in parts.values())
    centre = rotation @ hub_points.mean(axis=0)
    origin = rotation.T @ np.array([centre[0], lowest, centre[2]])

    gltf = GltfWriter()
    gltf.materials = doc.get("materials", [])
    children: list[int] = []
    for part in parts.values():
        transform(part, rotation, origin, scale)
        children.append(gltf.mesh_node(part))
    for prop, hub in zip(props, hub_points, strict=True):
        transform(prop, rotation, origin, scale)
        translation = (rotation @ (hub - origin)) * scale
        prop.attributes["POSITION"] -= translation.astype(np.float32)
        prop.name = _prop_name(translation)
        spin = spin_direction(prop.attributes["POSITION"], prop.indices)
        if spin != ARDUPILOT_QUAD_X[prop.name]:
            raise GlbFormatError(f"{prop.name}: blades pitched to turn the other way")
        node = gltf.mesh_node(prop)
        gltf.nodes[node]["translation"] = [float(v) for v in translation]
        gltf.nodes[node]["extras"] = {"spin": spin}
        children.append(node)
    root = len(gltf.nodes)
    gltf.nodes.append({"name": "fpv-drone", "children": children})

    extra: dict[str, object] = {
        "images": gltf.images(doc, buffer, texture_px),
        "textures": doc.get("textures", []),
        "samplers": doc.get("samplers", []),
    }
    if "extensionsUsed" in doc:
        extra["extensionsUsed"] = doc["extensionsUsed"]
    asset = {
        "generator": "swarm.fpvmodel",
        "copyright": f'"FPV Drone" by PinaPipe, CC BY 4.0, {SOURCE_URL}',
    }
    return gltf.to_glb(root, asset, **extra)


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: python -m swarm.fpvmodel <sketchfab .glb> <out.glb>")
    out = Path(sys.argv[2])
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(build_glb(Path(sys.argv[1]).read_bytes()))
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
