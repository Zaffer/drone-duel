"""What content/*.toml holds: places, scenarios, vehicles, race courses and radio profiles.

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


Turbulence = Literal["none", "light", "moderate", "severe"]


class Wind(Strict):
    speed_ms: float = Field(ge=0, le=30, description="mean wind at 6 m, m/s")
    from_deg: float = Field(ge=0, lt=360, description="where it blows from, degrees true")
    turbulence: Turbulence = Field(description="MIL-F-8785C intensity")
    gusts: bool = Field(description="random 1-cos gusts on top")


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


Mode = Literal["freefly", "duel", "race"]


class Scenario(Strict):
    id: str
    name: str
    mode: Mode
    location: str
    vehicles: list[str] = Field(min_length=1, description="what the vehicle picker offers")
    default_vehicle: str
    course: str | None = Field(default=None, description="race: the course id")
    blurb: str = ""

    @model_validator(mode="after")
    def _check(self) -> "Scenario":
        if self.default_vehicle not in self.vehicles:
            raise ValueError(f"default_vehicle {self.default_vehicle!r} is not in vehicles")
        if (self.mode == "race") != (self.course is not None):
            raise ValueError("a race scenario names a course; no other mode does")
        return self


FirmwareKind = Literal["ardupilot-copter", "inav", "none"]
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


class Gate(Strict):
    at: LatLon
    alt_m: float
    ypr_deg: tuple[float, float, float] = Field(description="yaw true, pitch, roll")
    size_m: tuple[float, float] = Field(description="width, height (inside the frame)")


class Course(Strict):
    id: str
    name: str
    location: str
    laps: int = Field(ge=1, le=20)
    alt_ref: Literal["agl", "amsl"]
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
