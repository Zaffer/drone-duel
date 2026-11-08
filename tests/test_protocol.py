"""The server ⇄ page protocol: the schema the page's types come from is current, and messages
validate as the page sends them."""

import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from drone_duels.server.protocol import POSE, POSE_FIELDS, client_msg, schema

SCHEMA = Path(__file__).resolve().parents[1] / "web" / "src" / "net" / "protocol.schema.json"


def test_the_page_schema_is_current() -> None:
    assert SCHEMA.read_text() == json.dumps(schema(), indent=2) + "\n", (
        "stale: run `uv run duels gen-protocol`"
    )


def test_page_messages_validate() -> None:
    m = client_msg.validate_json('{"ch":"rc","seq":3,"us":[1500,1500,1000,1500,1000]}')
    assert m.ch == "rc"
    m = client_msg.validate_json(
        '{"ch":"session","action":"start","location":"billy-bishop","vehicle":"quad5","pad":0}'
    )
    assert m.ch == "session"
    with pytest.raises(ValidationError):
        client_msg.validate_json('{"ch":"rc","seq":1,"us":[1500]}')
    with pytest.raises(ValidationError):
        client_msg.validate_json('{"ch":"session","action":"explode"}')


def test_target_and_hit_messages_validate() -> None:
    m = client_msg.validate_json('{"ch":"target","action":"hover","height_m":30}')
    assert m.ch == "target"
    with pytest.raises(ValidationError):
        client_msg.validate_json('{"ch":"target","action":"hover","height_m":500}')
    with pytest.raises(ValidationError):  # hover needs a height
        client_msg.validate_json('{"ch":"target","action":"hover"}')
    go = client_msg.validate_json(
        '{"ch":"target","action":"go","lat":43.6,"lon":-79.4,"height_m":-3,"vel_ned":[1,2,0]}'
    )
    assert go.ch == "target" and go.vel_ned == (1.0, 2.0, 0.0)
    for bad in (
        '{"ch":"target","action":"go","height_m":10}',
        '{"ch":"target","action":"go","vel_ned":[5,0,1]}',
    ):
        with pytest.raises(ValidationError):  # go needs a point on the path
            client_msg.validate_json(bad)
    path = client_msg.validate_json(
        '{"ch":"target","action":"path","height_m":20,"points":[[43.6,-79.4],[43.61,-79.4]]}'
    )
    assert path.ch == "target" and path.points == [(43.6, -79.4), (43.61, -79.4)]
    two = '[43.6,-79.4],[43.61,-79.4]'
    for bad in (
        '{"ch":"target","action":"path","height_m":20,"points":[[43.6,-79.4]]}',  # one point
        '{"ch":"target","action":"path","height_m":20}',  # none
        f'{{"ch":"target","action":"path","height_m":1,"points":[{two}]}}',  # too low
        '{"ch":"target","action":"path","height_m":20,"points":[[95,-79.4],[43.6,-79.4]]}',
        f'{{"ch":"target","action":"path","height_m":20,"points":[{",".join([two] * 16)}]}}',
    ):
        with pytest.raises(ValidationError):
            client_msg.validate_json(bad)
    hit = (
        '{"ch":"hit","part":"%s","motor":%s,"impulse_ned":[0.5,0,0],"at_frd":[0.1,0.1,0]}'
    )
    m = client_msg.validate_json(hit % ("arm", "2"))
    assert m.ch == "hit" and m.motor == 2 and m.impulse_ned == (0.5, 0.0, 0.0)
    for part in ("battery", "body", "payload"):
        assert client_msg.validate_json(hit % (part, "null")).ch == "hit"
    with pytest.raises(ValidationError):  # an arm names its motor
        client_msg.validate_json(hit % ("arm", "null"))
    with pytest.raises(ValidationError):
        client_msg.validate_json(hit % ("arm", "5"))
    with pytest.raises(ValidationError):
        client_msg.validate_json(hit % ("prop", "1"))


def test_pose_layout() -> None:
    assert POSE.size == 8 * len(POSE_FIELDS) == 144
    assert POSE_FIELDS[:4] == ("t", "lat", "lon", "alt")
