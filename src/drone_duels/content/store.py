"""Load content/*.toml into the schema's models, and check what refers to what."""

import tomllib
from dataclasses import dataclass
from pathlib import Path

from pydantic import BaseModel, ValidationError

from drone_duels.content.schema import Controller, Course, Location, Payload, Vehicle, Weapon

ROOT = Path(__file__).resolve().parents[3]
CONTENT = ROOT / "content"


class ContentError(ValueError):
    pass


def _load[M: BaseModel](model: type[M], path: Path) -> M:
    try:
        data = tomllib.loads(path.read_text())
    except tomllib.TOMLDecodeError as exc:
        raise ContentError(f"{path}: {exc}") from None
    data.setdefault("id", path.stem)
    if data["id"] != path.stem:
        raise ContentError(f"{path}: id {data['id']!r} must match the file name")
    try:
        return model.model_validate(data)
    except ValidationError as exc:
        lines = [
            f"  {'.'.join(str(p) for p in e['loc']) or '(file)'}: {e['msg']}" for e in exc.errors()
        ]
        raise ContentError(f"{path}:\n" + "\n".join(lines)) from None


def _load_dir[M: BaseModel](model: type[M], directory: Path) -> dict[str, M]:
    return {p.stem: _load(model, p) for p in sorted(directory.glob("*.toml"))}


@dataclass(frozen=True)
class Content:
    locations: dict[str, Location]
    vehicles: dict[str, Vehicle]
    courses: dict[str, Course]
    controllers: dict[str, Controller]
    weapons: dict[str, Weapon]
    payloads: dict[str, Payload]

    def location(self, location_id: str) -> Location:
        try:
            return self.locations[location_id]
        except KeyError:
            raise ContentError(f"no location {location_id!r}") from None

    def vehicle(self, vehicle_id: str) -> Vehicle:
        try:
            return self.vehicles[vehicle_id]
        except KeyError:
            raise ContentError(f"no vehicle {vehicle_id!r}") from None

    def payload(self, payload_id: str) -> Payload:
        try:
            return self.payloads[payload_id]
        except KeyError:
            raise ContentError(f"no payload {payload_id!r}") from None

    def fitted(self, vehicle_id: str, payload_id: str | None) -> Vehicle:
        """The vehicle, with the payload fitted (its mass, its parts, its firmware's files after
        the vehicle's), if one is named."""
        vehicle = self.vehicle(vehicle_id)
        if payload_id is None:
            return vehicle
        payload = self.payload(payload_id)
        extra = payload.params.get(vehicle.firmware.kind, [])
        firmware = vehicle.firmware.model_copy(
            update={"params": [*vehicle.firmware.params, *extra]}
        )
        return vehicle.model_copy(update={"payload": payload.spec(), "firmware": firmware})


def load(directory: Path = CONTENT) -> Content:
    content = Content(
        locations=_load_dir(Location, directory / "locations"),
        vehicles=_load_dir(Vehicle, directory / "vehicles"),
        courses=_load_dir(Course, directory / "courses"),
        controllers=_load_dir(Controller, directory / "controllers"),
        weapons=_load_dir(Weapon, directory / "weapons"),
        payloads=_load_dir(Payload, directory / "payloads"),
    )
    _check_references(content)
    return content


def _check_references(c: Content) -> None:
    problems: list[str] = []
    for course in c.courses.values():
        if course.location not in c.locations:
            problems.append(f"course {course.id}: no location {course.location!r}")
    for vehicle in c.vehicles.values():
        if vehicle.payload is not None:
            problems.append(f"vehicle {vehicle.id}: a payload is fitted by a mode, not the file")
    if problems:
        raise ContentError("\n".join(problems))
