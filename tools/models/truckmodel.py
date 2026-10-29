"""Sketchfab's "Military Truck" by Raphael Escamilla (CC BY 4.0) as the COP's point B vehicle.

assets/military_truck.glb is the Sketchfab download as it came (GLB, 1k textures), committed
like the drone's (tools.models.fpvmodel). After changing this converter, rebuild the model the COP
serves with

    uv run python -m tools.models.truckmodel assets/military_truck.glb \\
        web/public/models/military-truck.glb

The source is already in metres, 4.9 m bumper to bumper. This bakes the node transforms into
the vertices and turns the truck so its front faces glTF +Z with +Y up, centred on the ground.
Its 60 parts merge into one mesh per material, a few draw calls instead of 60.

Two of the source's materials are alpha blended, the body's among them, though only the glass
is see-through. Cesium draws blended surfaces without hiding what is behind them, so the seats
would show through the doors. Those materials are alpha masked instead, and the glass gets a
blended copy of its material. The textures scale down to TEXTURE_PX.
"""

import sys
from pathlib import Path
from typing import Any

import numpy as np

from tools.models.gltf import (
    GlbFormatError,
    GltfWriter,
    Primitive,
    read_glb,
    scene_primitives,
    transform,
    yaw,
)

SOURCE_URL = "https://sketchfab.com/3d-models/military-truck-d3a85af5faef4fddb217b3fce073614c"
FRONT_XZ = (-1.0, 0.0)
"""The source's front (grille, headlights, bonnet) is -X, found by inspection."""
GLASS_NODES = {
    "frontwindow_low.001",
    "frontrigthw_low.001",
    "Passwindows_low.001",
    "windowback_low.001",
    "backdoorwin_low.001",
}
"""The source's window parts. Their material also paints the seats, tyres and lights."""
TEXTURE_PX = 512
"""Largest texture side. The truck is a few hundred pixels long at most, even close up."""


def truck_parts(doc: dict[str, Any], buffer: bytes) -> list[Primitive]:
    """The source's parts, front on +Z, centred on the ground, each named after its node."""
    parts = scene_primitives(doc, buffer)
    rotation = yaw(np.array(FRONT_XZ))
    points = np.vstack([p.attributes["POSITION"] for p in parts]) @ rotation.T
    low, high = points.min(axis=0), points.max(axis=0)
    origin = rotation.T @ np.array([(low[0] + high[0]) / 2, low[1], (low[2] + high[2]) / 2])
    for part in parts:
        transform(part, rotation, origin, 1.0)
        # The mesh nodes are all "defaultMaterial"; their parents carry the part names.
        part.name = part.nodes[-2]
    return parts


def merge(parts: list[Primitive], name: str) -> Primitive:
    """One primitive holding every triangle of `parts`, which share a material."""
    keys = parts[0].attributes.keys()
    if any(p.attributes.keys() != keys for p in parts):
        raise GlbFormatError(f"{name}: parts with different vertex attributes")
    offsets = np.cumsum([0] + [len(p.attributes["POSITION"]) for p in parts[:-1]])
    return Primitive(
        name,
        {k: np.concatenate([p.attributes[k] for p in parts]) for k in keys},
        np.concatenate([p.indices + o for p, o in zip(parts, offsets, strict=True)]).astype(
            np.uint32
        ),
        parts[0].material,
    )


def build_glb(source: bytes, texture_px: int | None = TEXTURE_PX) -> bytes:
    doc, buffer = read_glb(source)
    parts = truck_parts(doc, buffer)
    glass = {p.name for p in parts} & GLASS_NODES
    if glass != GLASS_NODES:
        raise GlbFormatError(f"no window parts {sorted(GLASS_NODES - glass)}")

    gltf = GltfWriter()
    sources: list[dict[str, object]] = doc.get("materials", [])
    gltf.materials = [
        {**m, "alphaMode": "MASK"} if m.get("alphaMode") == "BLEND" else m for m in sources
    ]
    groups: dict[tuple[int | None, bool], list[Primitive]] = {}
    for part in parts:
        groups.setdefault((part.material, part.name in GLASS_NODES), []).append(part)
    children: list[int] = []
    for (material, is_glass), members in groups.items():
        if material is None:
            raise GlbFormatError(f"{members[0].name}: no material")
        name = str(sources[material].get("name", f"material{material}"))
        if is_glass:
            gltf.materials.append({**sources[material], "name": "glass", "alphaMode": "BLEND"})
            material, name = len(gltf.materials) - 1, "glass"
        merged = merge(members, name)
        merged.material = material
        children.append(gltf.mesh_node(merged))
    root = len(gltf.nodes)
    gltf.nodes.append({"name": "military-truck", "children": children})

    extra: dict[str, object] = {
        "images": gltf.images(doc, buffer, texture_px),
        "textures": doc.get("textures", []),
        "samplers": doc.get("samplers", []),
    }
    asset = {
        "generator": "tools.models.truckmodel",
        "copyright": f'"Military Truck" by Raphael Escamilla, CC BY 4.0, {SOURCE_URL}',
    }
    return gltf.to_glb(root, asset, **extra)


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: python -m tools.models.truckmodel <sketchfab .glb> <out.glb>")
    out = Path(sys.argv[2])
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(build_glb(Path(sys.argv[1]).read_bytes()))
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
