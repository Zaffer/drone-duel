"""Just enough binary FBX reading for the operator's model (tools.models.operatormodel).

An FBX file is a tree of records, each a name, a list of typed values and its children. The
scene is in two of them: Objects (models, geometry, materials, skin deformers) and Connections
(which object belongs to which). This reads skinned meshes and their skeleton as they stand in
the bind pose; animation curves, textures and everything else are left alone.
"""

import struct
import zlib
from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np

ARRAY_TYPES = {b"f": "<f4", b"d": "<f8", b"l": "<i8", b"i": "<i4", b"b": "u1"}
SCALAR_TYPES = {b"Y": "<h", b"C": "?", b"I": "<i", b"F": "<f", b"D": "<d", b"L": "<q"}


class FbxFormatError(ValueError):
    pass


@dataclass
class Record:
    name: str
    values: list[object]
    children: list["Record"] = field(default_factory=list)

    def child(self, name: str) -> "Record":
        for c in self.children:
            if c.name == name:
                return c
        raise FbxFormatError(f"{self.name} has no {name}")

    def array(self, name: str) -> np.ndarray:
        value = self.child(name).values[0]
        if not isinstance(value, np.ndarray):
            raise FbxFormatError(f"{self.name}.{name} is not an array")
        return value

    def text(self, name: str) -> str:
        return str(self.child(name).values[0])

    def properties(self) -> dict[str, list[object]]:
        """The Properties70 values by name (each a P record: name, type, label, flags, values)."""
        try:
            block = self.child("Properties70")
        except FbxFormatError:
            return {}
        return {str(p.values[0]): p.values[4:] for p in block.children}


def read_records(data: bytes) -> list[Record]:
    """The top-level records of a binary FBX file."""
    if not data.startswith(b"Kaydara FBX Binary  \0"):
        raise FbxFormatError("not a binary FBX file")
    (version,) = struct.unpack_from("<I", data, 23)
    header = "<QQQ" if version >= 7500 else "<III"
    header_size = struct.calcsize(header)

    def value(at: int) -> tuple[object, int]:
        kind = data[at : at + 1]
        at += 1
        if kind in SCALAR_TYPES:
            fmt = SCALAR_TYPES[kind]
            return struct.unpack_from(fmt, data, at)[0], at + struct.calcsize(fmt)
        if kind in ARRAY_TYPES:
            count, encoding, size = struct.unpack_from("<III", data, at)
            raw = data[at + 12 : at + 12 + size]
            if encoding == 1:
                raw = zlib.decompress(raw)
            return np.frombuffer(raw, ARRAY_TYPES[kind], count), at + 12 + size
        if kind in (b"S", b"R"):
            (size,) = struct.unpack_from("<I", data, at)
            raw = data[at + 4 : at + 4 + size]
            return (raw.decode("utf-8", "replace") if kind == b"S" else raw), at + 4 + size
        raise FbxFormatError(f"unknown value type {kind!r} at {at - 1}")

    def record(at: int) -> tuple[Record | None, int]:
        end, count, _ = struct.unpack_from(header, data, at)
        if end == 0:  # the empty record that closes a list of children
            return None, at + header_size + 1
        name_size = data[at + header_size]
        at += header_size + 1
        node = Record(data[at : at + name_size].decode(), [])
        at += name_size
        for _ in range(count):
            v, at = value(at)
            node.values.append(v)
        while at < end:
            child, at = record(at)
            if child is None:
                break
            node.children.append(child)
        return node, end

    records: list[Record] = []
    at = 27
    while at < len(data) - header_size:
        node, at = record(at)
        if node is None:
            break
        records.append(node)
    return records


@dataclass
class Bone:
    name: str
    parent: str | None
    """The nearest ancestor that is a bone with a bind matrix, if any."""
    bind: np.ndarray
    """(4, 4) its global transform in the bind pose."""


@dataclass
class SkinnedMesh:
    name: str
    points: np.ndarray
    """(n, 3) the control points in the bind pose, in the scene's frame."""
    triangles: np.ndarray
    """(m, 3) control point indices; polygons are fanned into triangles."""
    normals: np.ndarray
    """(m, 3, 3) each triangle corner's normal in the bind pose."""
    uvs: np.ndarray
    """(m, 3, 2) each triangle corner's texture coordinates, FBX's way up (v from the bottom)."""
    materials: list[str]
    """Each triangle's material name."""
    weights: dict[str, tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)
    """Bone name to (control point indices, weights)."""


def _id(record: Record) -> int:
    id = record.values[0]
    if not isinstance(id, int):
        raise FbxFormatError(f"{record.name}: no object id")
    return id


def _name(record: Record) -> str:
    return str(record.values[1]).split("\0")[0]


def _corner_index(layer: Record, index: str, corners: np.ndarray) -> np.ndarray:
    """Where each polygon corner's value is in a layer element's data (normals, UVs). `corners` are
    the corners' control points; `index` names the element's index array, if it has one."""
    mapping = layer.text("MappingInformationType")
    if mapping == "ByPolygonVertex":
        at = np.arange(len(corners))
    elif mapping in ("ByVertice", "ByVertex", "ByControlPoint"):
        at = corners
    else:
        raise FbxFormatError(f"{layer.name}: mapping {mapping} not supported")
    if layer.text("ReferenceInformationType") == "IndexToDirect":
        return layer.array(index)[at]
    return at


class Scene:
    """A binary FBX file's objects and the connections between them."""

    def __init__(self, data: bytes) -> None:
        records = {r.name: r for r in read_records(data)}
        self.objects = {_id(o): o for o in records["Objects"].children}
        self.children: dict[int, list[int]] = defaultdict(list)
        self.parents: dict[int, list[int]] = defaultdict(list)
        for c in records["Connections"].children:
            source, destination = c.values[1], c.values[2]
            if isinstance(source, int) and isinstance(destination, int):
                self.children[destination].append(source)
                self.parents[source].append(destination)

    def _connected(self, id: int, kind: str, up: bool = False) -> list[Record]:
        ids = self.parents[id] if up else self.children[id]
        return [self.objects[i] for i in ids if i in self.objects and self.objects[i].name == kind]

    def bones(self) -> dict[str, Bone]:
        """Every bone that some skin binds to, with its bind pose (the clusters' TransformLink)."""
        binds: dict[int, np.ndarray] = {}
        for id, o in self.objects.items():
            if o.name == "Deformer" and o.values[2] == "Cluster":
                (bone,) = self._connected(id, "Model")
                binds[_id(bone)] = o.array("TransformLink").reshape(4, 4).T
        out: dict[str, Bone] = {}
        for id, matrix in binds.items():
            parent = None
            up = self.parents[id]
            while up:
                above = up[0]
                if above in binds:
                    parent = _name(self.objects[above])
                    break
                up = self.parents[above] if above in self.objects else []
            out[_name(self.objects[id])] = Bone(_name(self.objects[id]), parent, matrix)
        return out

    def meshes(self) -> list[SkinnedMesh]:
        """Every mesh model's geometry, in the bind pose, with its skin weights."""
        out: list[SkinnedMesh] = []
        for id, model in self.objects.items():
            if model.name != "Model" or model.values[2] != "Mesh":
                continue
            (geometry,) = self._connected(id, "Geometry")
            materials = [_name(m) for m in self._connected(id, "Material")]
            out.append(self._mesh(_name(model), geometry, materials, _geometric(model)))
        return out

    def _mesh(
        self, name: str, geometry: Record, materials: list[str], geometric: np.ndarray
    ) -> SkinnedMesh:
        points = geometry.array("Vertices").reshape(-1, 3).astype(np.float64)
        polygon = geometry.array("PolygonVertexIndex").astype(np.int64)
        ends = polygon < 0
        corners = np.where(ends, ~polygon, polygon)
        normal_layer = geometry.child("LayerElementNormal")
        normals = normal_layer.array("Normals").reshape(-1, 3)
        normal_at = _corner_index(normal_layer, "NormalsIndex", corners)
        uv_layer = geometry.child("LayerElementUV")
        uvs = uv_layer.array("UV").reshape(-1, 2)
        uv_at = _corner_index(uv_layer, "UVIndex", corners)
        material_layer = geometry.child("LayerElementMaterial")
        slots = material_layer.array("Materials").astype(np.int64)
        by_polygon = material_layer.text("MappingInformationType") != "AllSame"

        # Fan each polygon from its first corner.
        starts = np.concatenate([[0], np.flatnonzero(ends)[:-1] + 1])
        sizes = np.flatnonzero(ends) - starts + 1
        first = np.repeat(starts, sizes - 2)
        step = np.concatenate([np.arange(1, s - 1) for s in sizes])
        fan = np.stack([first, first + step, first + step + 1], axis=1)
        polygon_of = np.repeat(np.arange(len(starts)), sizes - 2)
        slot = slots[polygon_of] if by_polygon else np.full(len(fan), slots[0])

        # The mesh's global transform at bind time: each cluster holds its bone's (TransformLink)
        # and the mesh's relative to that bone (Transform).
        bind = np.eye(4)
        weights: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for skin in self._connected(_id(geometry), "Deformer"):
            for cluster in self._connected(_id(skin), "Deformer"):
                link = cluster.array("TransformLink").reshape(4, 4).T
                bind = link @ cluster.array("Transform").reshape(4, 4).T
                (bone,) = self._connected(_id(cluster), "Model")
                try:
                    indices = cluster.array("Indexes").astype(np.int64)
                    values = cluster.array("Weights").astype(np.float64)
                except FbxFormatError:
                    continue  # a cluster that moves nothing
                weights[_name(bone)] = (indices, values)
        bind = bind @ geometric
        linear = bind[:3, :3]
        return SkinnedMesh(
            name,
            points @ linear.T + bind[:3, 3],
            corners[fan],
            _unit(normals[normal_at][fan] @ np.linalg.inv(linear)),
            uvs[uv_at][fan].astype(np.float64),
            [materials[s] for s in slot],
            weights,
        )


def euler_xyz(degrees: list[float]) -> np.ndarray:
    """The rotation of FBX's default XYZ Euler angles: about X first, then Y, then Z."""
    x, y, z = np.radians(degrees)
    rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
    ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
    return rz @ ry @ rx


def _geometric(model: Record) -> np.ndarray:
    """A model's geometric transform: an offset of its geometry alone, which its children and
    the skin's matrices leave out."""
    p = model.properties()

    def floats(key: str, default: float) -> list[float]:
        return [float(v) for v in p[key][:3]] if key in p else [default] * 3  # pyright: ignore[reportArgumentType]

    m = np.eye(4)
    m[:3, :3] = euler_xyz(floats("GeometricRotation", 0.0)) * floats("GeometricScaling", 1.0)
    m[:3, 3] = floats("GeometricTranslation", 0.0)
    return m


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-12)
