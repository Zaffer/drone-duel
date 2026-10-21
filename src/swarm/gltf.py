"""Just enough glTF 2.0 binary (GLB) reading and writing for the viewer's models.

swarm.fpvmodel and swarm.truckmodel read their Sketchfab sources with read_glb and
scene_primitives, and write the COP's models with GltfWriter.
"""

import io
import json
import struct
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np
from PIL import Image

ARRAY_BUFFER = 34962
ELEMENT_ARRAY_BUFFER = 34963
COMPONENT_TYPES: dict[int, type[np.generic]] = {
    5120: np.int8,
    5121: np.uint8,
    5122: np.int16,
    5123: np.uint16,
    5125: np.uint32,
    5126: np.float32,
}
COMPONENT_COUNTS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}
FLOAT_ATTRIBUTES = {"POSITION": "VEC3", "NORMAL": "VEC3", "TANGENT": "VEC4"}


class GlbFormatError(ValueError):
    pass


@dataclass
class Primitive:
    name: str
    attributes: dict[str, np.ndarray]
    """POSITION, NORMAL and TANGENT if present, and the TEXCOORD_n materials read; float32."""
    indices: np.ndarray
    """(m,) uint32, triangles."""
    material: int | None
    nodes: tuple[str, ...] = ()
    """The names of the nodes from the scene root down to the one holding the mesh."""


def read_glb(data: bytes) -> tuple[dict[str, Any], bytes]:
    """The JSON document and the binary chunk of a GLB."""
    magic, version, length = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF" or version != 2 or length > len(data):
        raise GlbFormatError("not a glTF 2.0 binary")
    chunks: dict[bytes, bytes] = {}
    at = 12
    while at < length:
        size, kind = struct.unpack_from("<I4s", data, at)
        chunks[kind] = data[at + 8 : at + 8 + size]
        at += 8 + size
    if b"JSON" not in chunks:
        raise GlbFormatError("GLB has no JSON chunk")
    return json.loads(chunks[b"JSON"]), chunks.get(b"BIN\0", b"")


def read_accessor(doc: dict[str, Any], buffer: bytes, index: int) -> np.ndarray:
    """Accessor `index` as a (count, components) array, honouring the view's byte stride."""
    accessor = doc["accessors"][index]
    if "sparse" in accessor or "bufferView" not in accessor:
        raise GlbFormatError(f"accessor {index}: sparse or bufferless accessors not supported")
    view = doc["bufferViews"][accessor["bufferView"]]
    dtype = np.dtype(COMPONENT_TYPES[accessor["componentType"]])
    components = COMPONENT_COUNTS[accessor["type"]]
    count = accessor["count"]
    element = dtype.itemsize * components
    stride = view.get("byteStride", element)
    start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    raw = np.frombuffer(buffer, np.uint8, stride * (count - 1) + element, start)
    rows = np.lib.stride_tricks.as_strided(raw, (count, element), (stride, 1))
    return np.ascontiguousarray(rows).view(dtype).reshape(count, components)


def _local(node: dict[str, Any]) -> np.ndarray:
    if "matrix" in node:
        return np.array(node["matrix"], dtype=float).reshape(4, 4).T
    x, y, z, w = node.get("rotation", (0.0, 0.0, 0.0, 1.0))
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    m = np.eye(4)
    m[:3, :3] = rotation * np.array(node.get("scale", (1.0, 1.0, 1.0)))
    m[:3, 3] = node.get("translation", (0.0, 0.0, 0.0))
    return m


def unit(v: np.ndarray) -> np.ndarray:
    return v / np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-12)


def _texcoords_read(doc: dict[str, Any]) -> set[int]:
    """The TEXCOORD_n sets some material's texture reads."""
    used: set[int] = set()

    def visit(value: object) -> None:
        if isinstance(value, dict):
            if "index" in value:
                used.add(int(value.get("texCoord", 0)))
            for v in value.values():
                visit(v)
        elif isinstance(value, list):
            for v in value:
                visit(v)

    visit(doc.get("materials", []))
    return used


def scene_primitives(doc: dict[str, Any], buffer: bytes) -> list[Primitive]:
    """Every triangle primitive in the default scene, with its node transforms baked in."""
    texcoords = {f"TEXCOORD_{n}": "VEC2" for n in _texcoords_read(doc)}
    out: list[Primitive] = []

    def walk(index: int, parent: np.ndarray, path: tuple[str, ...]) -> None:
        node = doc["nodes"][index]
        world = parent @ _local(node)
        path = (*path, node.get("name", ""))
        if "mesh" in node:
            mesh = doc["meshes"][node["mesh"]]
            for primitive in mesh["primitives"]:
                baked = _bake(doc, buffer, primitive, mesh.get("name", ""), world, texcoords)
                baked.nodes = path
                out.append(baked)
        for child in node.get("children", []):
            walk(child, world, path)

    for root in doc["scenes"][doc.get("scene", 0)]["nodes"]:
        walk(root, np.eye(4), ())
    return out


def _bake(
    doc: dict[str, Any],
    buffer: bytes,
    primitive: dict[str, Any],
    name: str,
    world: np.ndarray,
    texcoords: dict[str, str],
) -> Primitive:
    if primitive.get("mode", 4) != 4 or "indices" not in primitive:
        raise GlbFormatError(f"{name}: only indexed triangles are supported")
    attributes: dict[str, np.ndarray] = {}
    for key in (*FLOAT_ATTRIBUTES, *texcoords):
        if key in primitive["attributes"]:
            index = primitive["attributes"][key]
            if doc["accessors"][index]["componentType"] != 5126:
                raise GlbFormatError(f"{name}: {key} is not float")
            attributes[key] = read_accessor(doc, buffer, index).astype(np.float64)
    linear = world[:3, :3]
    flip = np.linalg.det(linear) < 0
    attributes["POSITION"] = attributes["POSITION"] @ linear.T + world[:3, 3]
    if "NORMAL" in attributes:
        attributes["NORMAL"] = unit(attributes["NORMAL"] @ np.linalg.inv(linear))
    if "TANGENT" in attributes:
        tangent = attributes["TANGENT"]
        handedness = tangent[:, 3:] * (-1.0 if flip else 1.0)
        attributes["TANGENT"] = np.hstack([unit(tangent[:, :3] @ linear.T), handedness])
    triangles = read_accessor(doc, buffer, primitive["indices"]).reshape(-1, 3)
    if flip:  # a mirroring transform turns the faces inside out
        triangles = triangles[:, ::-1]
    return Primitive(
        name,
        {k: v.astype(np.float32) for k, v in attributes.items()},
        triangles.reshape(-1).astype(np.uint32),
        primitive.get("material"),
    )


def yaw(direction_xz: np.ndarray) -> np.ndarray:
    """Rotation about +Y that turns `direction_xz` (x, z) onto +Z."""
    angle = -float(np.arctan2(direction_xz[0], direction_xz[1]))
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def transform(primitive: Primitive, rotation: np.ndarray, origin: np.ndarray, scale: float) -> None:
    """Move `origin` to the origin, then rotate and scale, in place."""
    a = primitive.attributes
    a["POSITION"] = ((a["POSITION"].astype(np.float64) - origin) @ rotation.T * scale).astype(
        np.float32
    )
    for key in ("NORMAL", "TANGENT"):
        if key in a:
            a[key][:, :3] = a[key][:, :3] @ rotation.T.astype(np.float32)


def shrink_image(data: bytes, texture_px: int | None) -> bytes:
    """The image with its longer side at most `texture_px`, in its own format."""
    if texture_px is None:
        return data
    image = Image.open(io.BytesIO(data))
    if max(image.size) <= texture_px:
        return data
    factor = texture_px / max(image.size)
    size = (max(1, round(image.width * factor)), max(1, round(image.height * factor)))
    out = io.BytesIO()
    image.resize(size, Image.Resampling.LANCZOS).save(out, format=image.format, optimize=True)
    return out.getvalue()


class GltfWriter:
    """Accumulates buffer views, accessors, meshes, materials and nodes into one GLB."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.views: list[dict[str, object]] = []
        self.accessors: list[dict[str, object]] = []
        self.meshes: list[dict[str, object]] = []
        self.materials: list[dict[str, object]] = []
        self.nodes: list[dict[str, object]] = []

    def view(self, data: bytes, target: int | None) -> int:
        self.buffer.extend(b"\0" * (-len(self.buffer) % 4))
        view: dict[str, object] = {"buffer": 0, "byteOffset": len(self.buffer)}
        view["byteLength"] = len(data)
        if target is not None:
            view["target"] = target
        self.buffer.extend(data)
        self.views.append(view)
        return len(self.views) - 1

    def accessor(self, array: np.ndarray, kind: str, target: int | None, bounds: bool) -> int:
        """`array` is uint32 or float32."""
        component = 5125 if array.dtype == np.uint32 else 5126
        accessor: dict[str, object] = {
            "bufferView": self.view(array.tobytes(), target),
            "componentType": component,
            "count": int(array.shape[0]),
            "type": kind,
        }
        if bounds:
            flat = array.reshape(array.shape[0], -1)
            accessor["min"] = [float(v) for v in flat.min(axis=0)]
            accessor["max"] = [float(v) for v in flat.max(axis=0)]
        self.accessors.append(accessor)
        return len(self.accessors) - 1

    def mesh_node(self, part: Primitive) -> int:
        """A mesh of `part` alone and a node holding it, both named after it."""
        attributes = {}
        for key, values in part.attributes.items():
            kind = FLOAT_ATTRIBUTES.get(key, "VEC2")
            attributes[key] = self.accessor(values, kind, ARRAY_BUFFER, key == "POSITION")
        primitive: dict[str, object] = {
            "attributes": attributes,
            "indices": self.accessor(part.indices, "SCALAR", ELEMENT_ARRAY_BUFFER, False),
        }
        if part.material is not None:
            primitive["material"] = part.material
        self.meshes.append({"name": part.name, "primitives": [primitive]})
        self.nodes.append({"name": part.name, "mesh": len(self.meshes) - 1})
        return len(self.nodes) - 1

    def images(
        self, doc: dict[str, Any], buffer: bytes, texture_px: int | None
    ) -> list[dict[str, object]]:
        """`doc`'s embedded images, shrunk to `texture_px`, as this GLB's images."""
        out: list[dict[str, object]] = []
        for image in doc.get("images", []):
            view = doc["bufferViews"][image["bufferView"]]
            start = view.get("byteOffset", 0)
            data = shrink_image(buffer[start : start + view["byteLength"]], texture_px)
            out.append({"bufferView": self.view(data, None), "mimeType": image["mimeType"]})
        return out

    def to_glb(self, root: int, asset: Mapping[str, object], **extra: object) -> bytes:
        """The GLB with scene root node `root`. `extra` adds top-level properties (images,
        textures, samplers, extensionsUsed). Empty lists are left out, as glTF requires."""
        self.buffer.extend(b"\0" * (-len(self.buffer) % 4))
        doc = {
            "asset": {"version": "2.0", **asset},
            "scene": 0,
            "scenes": [{"nodes": [root]}],
            "nodes": self.nodes,
            "meshes": self.meshes,
            "materials": self.materials,
            "accessors": self.accessors,
            "bufferViews": self.views,
            "buffers": [{"byteLength": len(self.buffer)}],
            **extra,
        }
        doc = {k: v for k, v in doc.items() if v != []}
        text = json.dumps(doc, separators=(",", ":")).encode()
        text += b" " * (-len(text) % 4)
        total = 12 + 8 + len(text) + 8 + len(self.buffer)
        return b"".join(
            [
                struct.pack("<4sII", b"glTF", 2, total),
                struct.pack("<I4s", len(text), b"JSON"),
                text,
                struct.pack("<I4s", len(self.buffer), b"BIN\0"),
                bytes(self.buffer),
            ]
        )
