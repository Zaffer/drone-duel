"""The server: its REST API, and a page's session over /ws (the truck, which needs no firmware)."""

import json

from fastapi.testclient import TestClient

from drone_duels.server.app import create_app


def test_content_and_config() -> None:
    with TestClient(create_app()) as client:
        content = client.get("/api/content").json()
        assert "quad5" in content["vehicles"]
        assert content["scenarios"]["race-areaxo"]["course"] == "areaxo-loop"
        config = client.get("/api/config").json()
        assert config["qgc_port"] == 14550


def test_prefs_round_trip(tmp_path, monkeypatch) -> None:  # noqa: ANN001
    monkeypatch.setenv("DRONE_DUELS_PREFS", str(tmp_path / "prefs.json"))
    with TestClient(create_app()) as client:
        assert client.get("/api/prefs").json() == {}
        client.put("/api/prefs", json={"video": {"system": "digital"}})
        assert client.get("/api/prefs").json()["video"]["system"] == "digital"
        assert client.put("/api/prefs", content=b"[1]").status_code == 400


def test_osd_fonts() -> None:
    with TestClient(create_app()) as client:
        r = client.get("/osd/font/0.png")
        assert r.status_code in (200, 404)  # 404 only without external/ardupilot
        assert client.get("/osd/font/nope.png").status_code == 404


def test_a_page_starts_the_truck_over_ws() -> None:
    with TestClient(create_app()) as client, client.websocket_connect("/ws") as ws:
        first = json.loads(ws.receive_text())
        assert first["ch"] == "status" and first["state"] == "idle"
        ws.send_text(json.dumps({"ch": "session", "action": "start",
                                 "scenario": "freefly-toronto", "vehicle": "truck", "pad": 0}))
        ws.send_text('{"ch": "nonsense"}')  # ignored, logged
        states = []
        for _ in range(20):
            msg = json.loads(ws.receive_text())
            if msg["ch"] == "status":
                states.append(msg["state"])
                if msg["state"] == "ready":
                    break
        assert states[-1] == "ready"
        ws.send_text(json.dumps({"ch": "session", "action": "stop", "scenario": None,
                                 "vehicle": None, "pad": 0}))


def test_another_sites_page_cannot_open_the_socket() -> None:
    import pytest
    from starlette.websockets import WebSocketDisconnect

    with TestClient(create_app()) as client:
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect("/ws", headers={"origin": "https://evil.example"}) as ws:
                ws.receive_text()
        with client.websocket_connect("/ws", headers={"origin": "http://localhost:8600"}) as ws:
            assert json.loads(ws.receive_text())["ch"] == "status"
