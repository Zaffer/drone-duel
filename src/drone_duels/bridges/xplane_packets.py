"""INAV SITL's side of X-Plane's UDP protocol: what it asks for, what it writes, what it reads.

INAV (target/SITL/sim/xplane.c) sends to the simulator's port:
  RREF  b"RREF\\0", int32 frequency, int32 index, the dataref's name (413 bytes): send me this
        dataref, under this index, this often (0: stop)
  DREF  b"DREF\\0", float32 value, the dataref's name, space padded (509 bytes): set it; INAV's
        outputs, the motor and up to three servos, each loop
and reads back
  RREF  b"RREF," then (int32 index, float32 value) pairs: the values it asked for
All little endian. INAV runs its control loop once for each RREF packet it reads.
"""

import struct
from dataclasses import dataclass

REQUEST = struct.Struct("<4sxii")
"""An RREF request's head: tag, frequency, index; the name follows."""
WRITE = struct.Struct("<4sxf")
"""A DREF write's head: tag, value; the name follows."""
PAIR = struct.Struct("<if")


@dataclass(frozen=True)
class Subscribe:
    index: int
    frequency: int
    name: str


@dataclass(frozen=True)
class Write:
    name: str
    value: float


def _name(raw: bytes) -> str:
    return raw.split(b"\0", 1)[0].decode("ascii", "replace").strip()


def parse(packet: bytes) -> Subscribe | Write | None:
    """One packet INAV sent; None for anything else."""
    if packet.startswith(b"RREF") and len(packet) > REQUEST.size:
        _, freq, index = REQUEST.unpack_from(packet)
        return Subscribe(index, freq, _name(packet[REQUEST.size :]))
    if packet.startswith(b"DREF") and len(packet) > WRITE.size:
        _, value = WRITE.unpack_from(packet)
        return Write(_name(packet[WRITE.size :]), value)
    return None


def values_packet(values: dict[int, float]) -> bytes:
    """The RREF reply carrying `values`, by index."""
    return b"RREF," + b"".join(PAIR.pack(i, v) for i, v in values.items())


def request_packet(index: int, frequency: int, name: str) -> bytes:
    """An RREF request as INAV sends it (for tests and tools)."""
    return REQUEST.pack(b"RREF", frequency, index) + name.encode().ljust(400, b"\0")


def write_packet(name: str, value: float) -> bytes:
    """A DREF write as INAV sends it (for tests and tools)."""
    return WRITE.pack(b"DREF", value) + (name.encode() + b"\0").ljust(500, b" ")
