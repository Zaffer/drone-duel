"""What content/*.toml holds: places, vehicles, race courses, radio profiles, weapons and
payloads.

Each file is one model. Every key is checked: an unknown key or a missing one is an error naming
the file and the key, so a typo never becomes a silent default (DESIGN.md §5).
"""

from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

LatLon = Annotated[tuple[float, float], Field(description="[latitude, longitude], degrees")]


class Strict(BaseModel):
    model_config = ConfigDict(
        extra="forbid", frozen=True, json_schema_serialization_defaults_required=True
    )


class Wind(Strict):
    speed_ms: float = Field(ge=0, le=30, description="mean wind at 6 m, m/s")
    from_deg: float = Field(ge=0, lt=360, description="where it blows from, degrees true")
    turbulence: float = Field(
        ge=0,
        le=3,
        description="MIL-F-8785C's turbulence for this wind (σ_w 0.1 × the wind at 6 m) times "
        "this: 0 smooth air, 1 the standard's, 2 and 3 as rough as its moderate and severe",
    )
    eddy_scale: float = Field(
        ge=0.25,
        le=4,
        description="the turbulence's scale lengths times this: larger is slower, rolling air; "
        "1 is MIL-F-8785C's",
    )
    gust_factor: float = Field(
        ge=1,
        le=2.5,
        description="the strongest gust over the mean wind, as a weather report gives it; "
        "1: no gusts on top of the turbulence",
    )
    gust_every_s: float = Field(ge=2, le=120, description="mean time between gusts, s")


class StartPad(Strict):
    at: LatLon
    heading_deg: float = Field(ge=0, lt=360)


class Location(Strict):
    id: str
    name: str
    origin: LatLon
    ground_alt_m: float | Literal["auto"] = Field(
        description='ground height at the origin above sea level, or "auto" to look it up'
    )
    heading_deg: float = Field(ge=0, lt=360, description="the way the start pads face")
    pads: list[StartPad] = Field(
        min_length=1, description="where vehicles start; the first is player 1's"
    )
    wind: Wind
    blurb: str = ""


FirmwareKind = Literal["ardupilot-copter", "betaflight", "inav", "none"]
PhysicsKind = Literal["quad", "wing", "browser-truck"]
VehicleKind = Literal["multirotor", "fixedwing", "ground"]
Launch = Literal["ground", "hand", "drive"]


class FirmwareSpec(Strict):
    kind: FirmwareKind
    params: list[str] = Field(
        default_factory=list, description="parameter / CLI files, relative to firmware/"
    )


class PhysicsSpec(Strict):
    kind: PhysicsKind
    airframe: str = Field(description="the model's airframe id (racer5in, chupito, truck)")


class CameraSpec(Strict):
    tilt_deg: float = Field(ge=-30, le=80, description="up from the nose")
    hfov_deg: float = Field(gt=30, lt=170, description="the FPV camera's horizontal view")
    offset_m: tuple[float, float, float] = Field(description="forward, right, down of the CG")


class AuxSpec(Strict):
    """What the firmware does on each AUX channel (channel numbers 5-16)."""

    arm: int = Field(ge=5, le=16)
    mode: int | None = Field(default=None, ge=5, le=16)
    modes: list[str] = Field(
        default_factory=list, description="the mode switch's positions, low to high"
    )
    launch: int | None = Field(default=None, ge=5, le=16)


class PayloadSpec(Strict):
    """A charge strapped under a quad (Shoot Down fits it, whichever quad: content/payloads):
    carried as part of its mass, drawn under it, a part a bullet can hit. Hit, or (one time in
    two) when the quad comes down on the ground after a hit, it goes off
    (physics/quad/model.py)."""

    name: str
    mass_kg: float = Field(gt=0, le=2)
    size_m: tuple[float, float] = Field(
        description="a tube along the quad, forward: its length and diameter"
    )
    below_m: float = Field(ge=0, le=0.2, description="its axis below the quad's centre of mass")
    lethal_m: float = Field(
        gt=0, le=50, description="within this of where it goes off, a person is killed"
    )


class Vehicle(Strict):
    id: str
    name: str
    kind: VehicleKind
    firmware: FirmwareSpec
    physics: PhysicsSpec
    model: str = Field(description="the 3D model, under web/public/models or a /vendor path")
    camera: CameraSpec
    launch: Launch
    aux: AuxSpec | None = None
    dynamics_overlay: bool = False
    blurb: str = ""
    payload: PayloadSpec | None = Field(
        default=None,
        description="fitted by the session (a start's payload, content/payloads), not in its file",
    )


class Payload(PayloadSpec):
    """A payload a mode fits to the vehicle it starts (content/payloads/*.toml)."""

    id: str
    params: dict[FirmwareKind, list[str]] = Field(
        default_factory=dict,
        description="parameter / CLI files, after the vehicle's, for a firmware that carries it",
    )

    def spec(self) -> PayloadSpec:
        return PayloadSpec.model_validate(self.model_dump(exclude={"id", "params"}))


class GateFrame(Strict):
    """The course's gate: a square frame round the inside that is flown through."""

    inside_m: tuple[float, float] = Field(description="width, height of the way through")
    outside_m: tuple[float, float] = Field(description="width, height of the whole frame")
    depth_m: float = Field(gt=0, description="the frame's thickness along the way through")

    @model_validator(mode="after")
    def _frame_round_inside(self) -> "GateFrame":
        if not all(0 < i < o for i, o in zip(self.inside_m, self.outside_m, strict=True)):
            raise ValueError("the inside must be smaller than the outside, and both above 0")
        return self


class Gate(Strict):
    at: LatLon
    alt_m: float = Field(description="the gate's centre, m above the ground or sea (alt_ref)")
    ypr_deg: tuple[float, float, float] = Field(description="yaw true, pitch, roll")


class Course(Strict):
    id: str
    name: str
    location: str
    laps: int = Field(ge=1, le=20)
    alt_ref: Literal["agl", "amsl"]
    gate: GateFrame
    gates: list[Gate] = Field(min_length=2)


class AxisMap(Strict):
    axis: int = Field(ge=0, le=15, description="Gamepad API axis index")
    reverse: bool = False


class ButtonMap(Strict):
    button: int = Field(ge=0, le=31)


ChannelSource = AxisMap | ButtonMap


class Controller(Strict):
    id: str
    name: str
    match: list[str] = Field(
        min_length=1,
        description="substrings of the Gamepad API id ('Vendor: 1209 Product: 4f54') that pick "
        "this profile",
    )
    channels: list[ChannelSource] = Field(
        min_length=4, max_length=16, description="channel 1, 2, 3 ... in AETR order"
    )
    notes: str = ""


Sight = Literal["holo", "irons"]


class Cartridge(Strict):
    """The round a weapon fires: what the ballistics (web/src/shoot/ballistics.ts) and the recoil
    (web/src/shoot/recoil.ts) need of it."""

    name: str
    bullet_g: float = Field(gt=0, description="the bullet's mass, g")
    diameter_mm: float = Field(gt=0, description="the bullet's diameter, mm")
    length_mm: float = Field(gt=0, description="the bullet's length, mm (its spin stability)")
    bc_g7: float = Field(gt=0, lt=1, description="ballistic coefficient against G7, lb/in²")
    muzzle_ms: float = Field(gt=0, description="muzzle speed from this weapon's barrel, m/s")
    powder_g: float = Field(gt=0, description="the powder charge, g: its gas recoils too")
    tracer_every: int = Field(
        ge=0,
        description="a tracer every this many rounds as loaded; 0: none (the menu may change it)",
    )


class Weapon(Strict):
    id: str
    name: str
    model: str = Field(description="the first-person model, under web/public/models")
    mass_kg: float = Field(gt=0, description="loaded, with its sight")
    barrel_in: float = Field(gt=0)
    twist_in: float = Field(gt=0, description="the rifling's turn, inches per turn, right hand")
    magazine: int = Field(ge=1)
    reload_s: float = Field(gt=0, description="a magazine change, s")
    sights: list[Sight] = Field(min_length=1, description="the sights it has, the first fitted")
    zero_m: float = Field(gt=0, description="the range its sights are zeroed at, m")
    cartridge: Cartridge
    blurb: str = ""
