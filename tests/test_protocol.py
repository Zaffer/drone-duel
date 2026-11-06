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
    hit = (
        '{"ch":"hit","part":"%s","motor":%s,"impulse_ned":[0.5,0,0],"at_frd":[0.1,0.1,0]}'
    )
    m = client_msg.validate_json(hit % ("prop", "2"))
    assert m.ch == "hit" and m.motor == 2 and m.impulse_ned == (0.5, 0.0, 0.0)
    assert client_msg.validate_json(hit % ("battery", "null")).ch == "hit"
    with pytest.raises(ValidationError):  # a rotor's part names its motor
        client_msg.validate_json(hit % ("motor", "null"))
    with pytest.raises(ValidationError):
        client_msg.validate_json(hit % ("prop", "5"))


def test_pose_layout() -> None:
    assert POSE.size == 8 * len(POSE_FIELDS) == 144
    assert POSE_FIELDS[:4] == ("t", "lat", "lon", "alt")
