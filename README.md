# Drone Duels

A flight simulator game and practice sim. Every aircraft flies on its real flight controller
firmware in SITL, on physics we model ourselves, over real places drawn from Google
Photorealistic 3D Tiles:

| Vehicle | Firmware | Physics |
|---|---|---|
| 5" FPV quad | ArduCopter 4.7 SITL | SkyFlow-Dynamics' `racer5in` (UZH NeuroBEM/Agilicious), JAX on the CPU, through ArduPilot's JSON backend |
| TBS Chupito (flying wing) | INAV 9.1 SITL | our strips-over-a-vortex-lattice model (`drone_duels.physics.wing`), through X-Plane's protocol |
| Military truck | none | Rapier, on colliders made from the map's own 3D tiles |

Fly around, duel a friend (one strikes the truck from the air, the other drives it away), or race
through gates head to head. Switch vehicles at any time. Look through the FPV camera with the
firmware's own OSD, as analog NTSC over a 5.8 GHz link
([analog-vision](https://github.com/Synetic-Labs/Analog-Vision)) or clean digital. Set the wind,
turbulence and gusts live. Plan ArduPilot missions in QGroundControl and watch them flown.

## Setup

```bash
scripts/setup.sh          # uv env, ArduPilot + INAV SITL, mavlink-router, bun, the page
uv run duels doctor       # what is there and what is missing
```

Everything external lands in `external/` (git-ignored). For Google's 3D tiles put
`GOOGLE_MAPS_API_KEY=...` in `.env.local` (the key is restricted to port 8600); without one the
map is Esri imagery on Esri terrain. A Cesium ion token (`CESIUM_ION_TOKEN`) is the middle option.

## Play

```bash
uv run duels              # serves the game on http://localhost:8600 and opens Chrome
```

Main menu: three columns, picked in turn: the scenario (**Free Fly**, **Duel**, **Race**), the
location, the vehicle. Free Fly starts when you pick the vehicle; switch vehicles in flight from
the pause menu's Vehicle tab. Duel and Race then meet the other player in a room, under the
vehicles: one hosts and reads out the code, the other joins (WebRTC, peer to peer). Esc steps back
a column. **Options** and **Exit** (it stops the server too) are in the top right corner.

**Controllers.** A drone radio in USB joystick mode, through Chrome's Gamepad API or, when Chrome
can't see all its axes, read by the server through Linux. Profiles for the BETAFPV LiteRadio 2,
Radiomaster TX15 and T8L are built in (`content/controllers/`). Calibrate any other radio once in
Options → Controls. CH5 arms, CH6 is the flight mode, CH8 throws the wing. A standard gamepad works
too, and the keyboard (W/S throttle, arrows, Q/E yaw, Enter arm, M mode; WASD drives the truck).

**Keys.** Esc pause and options · V or 1–6 views (FPV, chase, far chase, spotter, map, orbit) ·
R reset · B vehicle · T throw (the wing) · D dynamics overlay (the wing) · N minimap · G a gust.

**ArduPilot practice.** QGroundControl connects on UDP 14550: plan and upload a mission, fly it
in AUTO, and watch it in the Map view.

## Content

Places, vehicles, race courses and radio profiles are TOML files in `content/`; add a
file to add one. Every key is checked, so a typo stops the game with its name.
`content/courses/*.toml` lists each gate's position, height, yaw/pitch/roll and size.

## Development

See `CLAUDE.md` for the rules and `DESIGN.md` for the architecture.

```bash
scripts/check.sh          # every check: ruff, basedpyright, pytest, tsc, biome, bun test, build
scripts/check.sh --fast   # without the tests that fly real firmware
cd web && bun run dev     # rebuild the page on every change
```
