# drone-duel

Python package `swarm` (src layout, uv). ArduPilot SITL, mavlink-router and
QGroundControl live in `external/` via `scripts/setup.sh`.

- Run everything with `uv run`. Never bare `python`. The sim is `uv run swarm sim` (fly it with
  `uv run swarm pilot`), or `uv run swarm fly` for both; `uv run swarm rehearse` flies the
  whole mission hands-free.
- After code changes run `uv run ruff check src tests`, `uv run basedpyright src tests`,
  `uv run pytest -q`.
- ArduCopter is 4.7.x. Use 4.7 parameter names (`MAV_SYSID`, `MAV_GCS_SYSID`,
  `LOIT_SPEED_MS`, `WP_SPD`). Check `external/ardupilot` source when unsure.
- The ground stack (`swarm pilot`, the watcher) is MAVLink system 254. QGroundControl is 255.
  Do not use 255.
- Sim and hardware differ only in `router/*.conf`. Keep it that way. The sim's stand-ins for
  links outside MAVLink are the exception: the USB radio (`swarm.radio`) and the viewer's
  camera feed, which draws SITL's physics (`swarm.physics`) as the camera's video, sent over
  a simulated analog video link (analog-vision's `analog.js`, vendored by `scripts/setup.sh`), and SITL's OSD over MSP DisplayPort
  (`swarm.osd`) as the analog OSD chip's; the wing's INAV OSD over MSP DisplayPort as the DJI
  air unit's (`swarm.wingrun.AirUnit`). The map, the FPV
  view and the dials stay on MAVLink.
- The mission's places and basic sim settings live in `mission.toml` (`swarm.settings`), not
  in code; CLI flags only override it.
- The fixed wing (the TBS Chupito) flies INAV, not ArduPilot: INAV SITL (`external/inav/`, via
  `scripts/setup.sh`) on our own flight model, `swarm.wingsim`, over X-Plane's UDP protocol
  (`uv run swarm wingsim`), drawn on the viewer's wing.html, which posts back the map's ground
  under it (`swarm.wingrun`). `swarm.wingsim` imports only itself and the standard library; keep it
  that way. Its aerodynamics are strips over a vortex lattice (`swarm/wingsim/strips.py`), its wind
  MIL-F-8785C's (`swarm/wingsim/weather.py`); its
  numbers' sources are in `swarm/wingsim/airframe.py`, and `tests/test_wingsim.py` holds the model
  to them. `swarm wingsim` records every flight to `logs/wingsim/`.
