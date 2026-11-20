# drone-duels

A flight simulator game and practice sim: real flight controller firmware in SITL (ArduPilot,
INAV, Betaflight) on our own physics, drawn over real places in Chrome. DESIGN.md is the
architecture, ROADMAP.md what comes next. Python package `drone_duels` (src layout, uv); the page
is TypeScript in `web/` (bun). ArduPilot, INAV and Betaflight SITL and mavlink-router live in
`external/` via `scripts/setup.sh`; Betaflight is built there with `scripts/betaflight-sitl.patch`
(change the patch, not the checkout).

- The web version (DESIGN.md §18) is the same page with no server: Betaflight's 5" quad and
  the truck, in Free Fly, Race and Duel. `scripts/setup-web.sh` builds its simulator
  (`external/web/sim/sim.wasm`: cudaflight's lockstep Betaflight with
  `scripts/cudaflight-web.patch` and `web/wasm/bfweb.c`, and the plant's generated C);
  `uv run duels web` builds the page into `web/dist-web/` and serves it. The page reaches its
  server only through `link` and `api` (`web/src/net/`); the web build swaps both for a worker
  (`web/src/sim/`, which imports nothing of the page's) and files. A change to the protocol,
  the session, the quad's model, the weather or Betaflight's MSP on the server has its twin in
  `web/src/sim/`. `uv run duels gen-web` rewrites what is generated for it (its tests'
  fixtures; `web/wasm/sfd.c`, never by hand).
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
- Places, vehicles, race courses and radio profiles are TOML in `content/`
  (`drone_duels.content`), not code. Every key is checked; add a file to add one.
- Every port is in `src/drone_duels/ports.py`. INAV and Betaflight must stay off 5760
  (mavlink-router connects there for ArduPilot) and Betaflight off 9001-9004, its own defaults.
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
- The page's engine (DESIGN.md §3.2, `web/src/engine/`): per-frame work joins the frame loop
  (`loop.each`/`loop.fixed`, named and timed); what moves or collides works in the place's local
  frame and asks `engine/ground.ts` (microseconds), never Cesium's `sampleHeight`/`pickPosition`
  (a render each; only `world.ts` may, metered); everything drawn goes through `world.add`;
  springs are `engine/spring.ts`; controls are tables mounted with `input/actions.ts`. A test
  fences these. `scripts/check.sh --perf` plays the page on the GPU and holds frames to budget,
  on the free map and Google's tiles (8600, costs money); `uvx --with playwright python
  scripts/perf.py --map free` is free and runs beside the game: run it when the frame's work
  changes.
- The page's state is alien-signals; anything over time (the WebSocket, gamepad samples, frames,
  timers, peer messages) is a native Chrome Observable (`web/src/types/observable.d.ts`).
  Chrome only. Colours come from `web/src/theme/tokens.css` ("Dusk").
- Keep AirMetal branding out of this repo.
