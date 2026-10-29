# drone-duels

A flight simulator game and practice sim: real flight controller firmware in SITL (ArduPilot,
INAV) on our own physics, drawn over real places in Chrome. DESIGN.md is the architecture,
ROADMAP.md what comes next. Python package `drone_duels` (src layout, uv); the page is TypeScript
in `web/` (bun). ArduPilot, INAV SITL and mavlink-router live in `external/` via
`scripts/setup.sh`.

- Run everything with `uv run`; never bare `python`. The game is `uv run duels` (serves the page
  on port 8600 and opens Chrome); `uv run duels doctor` says what is missing.
- No node here: use bun (`~/.bun/bin/bun`, `bunx`). `cd web && bun run build` builds the page;
  `bun run dev` rebuilds on change.
- After changes run `scripts/check.sh` (`--fast` skips the tests that start real firmware): ruff,
  basedpyright and pytest; tsc, biome, bun test and the page's build; and that the protocol's
  schema is current.
- The server ⇄ page protocol lives in `src/drone_duels/server/protocol.py` (pydantic). After
  changing it run `uv run duels gen-protocol`, which writes `web/src/net/protocol.schema.json` and
  the generated `web/src/net/protocol.ts` (never edit that by hand).
- Places, scenarios, vehicles, race courses and radio profiles are TOML in `content/`
  (`drone_duels.content`), not code. Every key is checked; add a file to add one.
- Every port is in `src/drone_duels/ports.py`. INAV must stay off 5760 (mavlink-router connects
  there for ArduPilot).
- ArduCopter is 4.7.x. Use 4.7 parameter names (`MAV_GCS_SYSID`, `LOIT_SPEED_MS`, `WP_SPD`).
  Check `external/ardupilot` source when unsure. The game is MAVLink system 254;
  QGroundControl is 255. Do not use 255.
- Two kinds of truth, kept apart (DESIGN.md §2.2): what the pilot sees (the camera, from the
  physics' true pose; the OSD, the firmware's own MSP DisplayPort) and what a ground station sees
  (the map view, the dials, plans: telemetry only, MAVLink or MSP).
- Heights: the sim's are above sea level, the map's above the WGS84 ellipsoid. The page measures
  `mapOffset` at the start pad (`web/src/world/geo.ts`); peers send map heights.
- `drone_duels.physics.core` and `physics.wing` import only themselves and the standard library
  (a test holds this). The quad uses SkyFlow-Dynamics' JAX backend as published, on the CPU.
  The wing's numbers' sources are in `physics/wing/airframe.py`, and `tests/test_physics_wing.py`
  holds the model to them.
- The page's state is alien-signals; anything over time (the WebSocket, gamepad samples, frames,
  timers, peer messages) is a native Chrome Observable (`web/src/types/observable.d.ts`).
  Chrome only. Colours come from `web/src/theme/tokens.css` ("Dusk").
- Keep AirMetal branding out of this repo.
