"""Load content/*.toml into the schema's models, and check what refers to what."""

import tomllib
from dataclasses import dataclass
from pathlib import Path

from pydantic import BaseModel, ValidationError

from drone_duels.content.schema import Controller, Course, Location, Scenario, Vehicle

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
    scenarios: dict[str, Scenario]
    vehicles: dict[str, Vehicle]
    courses: dict[str, Course]
    controllers: dict[str, Controller]

    def scenario(self, scenario_id: str) -> Scenario:
        try:
            return self.scenarios[scenario_id]
        except KeyError:
            raise ContentError(f"no scenario {scenario_id!r}") from None

    def vehicle(self, vehicle_id: str) -> Vehicle:
        try:
            return self.vehicles[vehicle_id]
        except KeyError:
            raise ContentError(f"no vehicle {vehicle_id!r}") from None


def load(directory: Path = CONTENT) -> Content:
    content = Content(
        locations=_load_dir(Location, directory / "locations"),
        scenarios=_load_dir(Scenario, directory / "scenarios"),
        vehicles=_load_dir(Vehicle, directory / "vehicles"),
        courses=_load_dir(Course, directory / "courses"),
        controllers=_load_dir(Controller, directory / "controllers"),
    )
    _check_references(content)
    return content


def _check_references(c: Content) -> None:
    problems: list[str] = []
    for s in c.scenarios.values():
        if s.location not in c.locations:
            problems.append(f"scenario {s.id}: no location {s.location!r}")
        problems += [
            f"scenario {s.id}: no vehicle {v!r}" for v in s.vehicles if v not in c.vehicles
        ]
        if s.course is not None:
            course = c.courses.get(s.course)
            if course is None:
                problems.append(f"scenario {s.id}: no course {s.course!r}")
            elif course.location != s.location:
                problems.append(f"scenario {s.id}: course {s.course} is at {course.location}")
    for course in c.courses.values():
        if course.location not in c.locations:
            problems.append(f"course {course.id}: no location {course.location!r}")
    if problems:
        raise ContentError("\n".join(problems))
