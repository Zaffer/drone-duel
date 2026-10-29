# Drone Duels: roadmap

After v1 (DESIGN.md §17). Roughly in priority order within each group.

## Vehicles and firmware

- **Betaflight SITL for the 5" quad**, as a firmware choice per vehicle. SkyFlow already closes
  the loop through Betaflight (`cudaflight`), and the JSON/MSP seams are the same shape.
- **ArduPilot Rover on the truck.** Rapier stays the physics (tile collisions). A lockstep link
  between the page and Rover SITL's JSON backend, for practising rover missions.
- **Our own 5" build** as a second SkyFlow-Dynamics parameter row (750 g, 6S, 2207 1950 KV), if
  racer5in flies too differently.
- More airframes: 7" long range, a cinewhoop, an ArduPlane fixed wing, an ArduPilot VTOL.
- INAV's `inav_xitl` battery datarefs, so the Chupito's pack is simulated, not a fixed 16.8 V.

## Swarm

- **Bring back the drone swarm** from Synetic-Labs/airmetal-swarm, branch `airmetal` (strip the
  AirMetal branding): the leader-follower launcher, ArduPilot FOLLOW, the geofence upload, the
  Logitech switch panel, the terminal panel and `rehearse`. It returns as a mode ("Swarm") with
  an AreaXO scenario, using the vehicle slots (DESIGN.md §3).
- Swarm strike in Duel: the pilot flies the swarm against the truck, as on `main`.

## Visualisation

- Dynamics overlay for the quad: per-rotor thrust and drag, inflow, frame drag, ground effect,
  from the plant's terms.
- Dynamics overlay for the truck: tyre forces and suspension.
- Digital video link effects: breakup, bitrate drop and latency for DJI, Walksnail and HDZero.
- DVR: record the FPV feed. Replays from flight records, with free cameras.

## Play

- In-game race course editor (place, drag and rotate gates on the map; save to TOML).
- Ghost of your best lap in a solo time trial.
- More than two players; spectators.
- More modes: tag, capture the flag, payload drop.
- More scenarios and locations.

## ArduPilot practice

- In-game mission planner: draw waypoints and upload them over MAVLink.
- Scenario scripts: failures to practise (GPS loss, failsafe, low battery, wind shear).
- INAV missions over MSP, drawn like ArduPilot's.

## Controllers and platforms

- Test and tune more radio profiles; a shared profile format others can contribute.
- Windows (WSL2) and macOS.
- A low-latency native input path (SDL2 in the server), if the Gamepad API's latency shows.

## Hardware

- Live mode: the Map/COP view against real aircraft through `router/field.conf`. The design
  keeps the ground-station side on telemetry only, so it needs no change (DESIGN.md §2.2).
