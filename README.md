# drone-duel

A pilot flies one drone. Up to four others follow it in formation, flown by ArduPilot's own
FOLLOW mode, and a Logitech switch panel runs the mission: launch, follow, pause, land, return
home. Each drone has its own controller. The same code flies ArduPilot SITL and real drones;
they differ only in `router/*.conf`.

## Architecture

```
copter 1..5 (ArduCopter 4.7)          field: Wi-Fi bridge of drone N -> UDP 14550+N
      |                               sim:   SITL serial0 -> TCP 5760+10i (or UDP, --router field)
      v
mavlink-router  (router/sitl.conf | router/field.conf)
      |-- UDP 14550  -> QGroundControl   sysid 255, optional backup display and control
      |-- UDP 14560  -> watcher          sysid 254, reads only: telemetry for the 3D viewer
      `-- UDP 14570  -> swarm pilot      sysid 254, switch panel and launcher (and the radio, sim)

watcher --JSON snapshots, UDP 14600--> swarm view --SSE--> browser, http://localhost:8600
```

- **Leader** = copter 1 (`--leader`), flown by the pilot.
- **Followers** = copters 2 to 5. `swarm pilot` (`swarm.launch`) arms each, lifts it off,
  sets its `FOLL_*` parameters and switches it to ArduPilot FOLLOW on the leader. From then on
  ArduPilot flies the formation, tracking the leader's position that mavlink-router relays to
  it; nothing on the ground is in that loop.
- **Formation** = `swarm.formation.formation_slots`: a ring 5 m **round** the leader
  (`follower_distance_m`), a square with four followers: copter 2 (Alpha) north, 3 (Bravo)
  east, 4 (Charlie) south, 5 (Delta) west. It is north-up (`FOLL_OFS_TYPE 0`), not tied to the
  leader's nose: when the leader yaws it turns on the spot and the followers stay put, turning
  with it (`FOLL_YAW_BEHAVE 2`). With them 90° apart, the leader's 120° FPV camera always has
  one or two in frame. Heights step 1.5 m up from the leader, the lowest: Alpha 1.5 m above
  it, Bravo 3 m, Charlie 4.5 m, Delta 6 m.
- **Watcher** = `swarm.watcher`: reads every drone and publishes what it sees for the 3D viewer;
  it never commands one. The leader is the drone the followers' `FOLL_SYSID` names; the role
  of each drone (leader, follower, pilot, watched) comes from its mode.
- **Readiness** follows the ArduPilot state machine, not timers: EKF position
  (`EKF_STATUS_REPORT`) before GUIDED, the `MAV_SYS_STATUS_PREARM_CHECK` bit
  before arming, `COMMAND_ACK` results are logged.
- **Avoidance** = ArduPilot's own (`AVD_*`, `sitl/swarm.parm`). Every drone takes the others'
  `GLOBAL_POSITION_INT`, which mavlink-router relays to it anyway (FOLLOW needs it), as
  obstacles (`DEV_OPTIONS 1`) and predicts the closest approach. When it would come within
  3 m across and 1.5 m up or down in the next 3 s (warning only inside 4 m), it escapes
  vertically for 2 s in AVOID_ADSB mode, the higher drone climbing and the lower descending,
  never below 5 m above home, then resumes FOLLOW, GUIDED or RTL. The formation already
  separates the drones by height, 1.5 m a step, so followers crossing at their own heights are
  no threat; it trips when a drone gets into another's height band, and resuming puts it back
  in its own band. `swarm pilot` sets the drone the radio flies to report only
  (`AVD_F_ACTION 1`): the pilot is never taken over, the others get out of its way. LAND is
  never interrupted, and the launcher does not re-send modes during an escape. Threats show
  in the terminal as `[2] too close 1: climbing or descending away (...)`.

## Radios and failsafes

Every drone has a controller of its own, bound and linked to its receiver for the whole
flight. The pilot flies one drone at a time, the one the switch panel selects (`swarm
pilot`, below), with that drone's controller; the others sit idle. ArduPilot's RC pre-arm
checks stay on, so a drone whose controller is off or unbound does not arm ("PreArm: RC not
found"). A drone whose controller drops out enters radio failsafe after `RC_FS_TIMEOUT`
(1 s): in GUIDED or FOLLOW it carries on (`FS_OPTIONS` bit 2), in a pilot mode it brakes
(`FS_THR_ENABLE 7`) until the link is back and the pilot picks a mode again.

In the sim, a USB transmitter (EdgeTX joystick mode) plugged into the laptop stands in for
whichever controller is in the pilot's hands: `swarm pilot` (below) streams it straight into
the selected copter's SITL RC input. Every other copter gets its own controller, idle: sticks
centred, throttle and switches low (`swarm.radio.RcLink`). A controller can be switched off
(`RcLink.power`, used by `swarm rehearse`): SITL on its own would keep the last RC input
forever, so after `RC_FS_TIMEOUT` (1 s) the copter gets failsafe throttle (900 µs, below
`FS_THR_VALUE`) and ArduPilot's real radio failsafe runs.

## Setup

```bash
scripts/setup.sh              # uv env, ArduPilot Copter-4.7.1 SITL, mavlink-router, 3D viewer libraries
WITH_QGC=1 scripts/setup.sh   # also QGroundControl, for `--qgc` (optional; nothing needs it)
```

Everything lands in `external/` (git-ignored). Builds use `gcc-14` where installed, else the
system compiler (gcc-15 builds Copter-4.7.1 fine); override with `CC`/`CXX`. meson and ninja
come from `uvx` when not installed, so no system packages or sudo are needed.

## Run the sim

```bash
uv run swarm fly        # copters, router, watcher, 3D viewer, radio, switch panel, keys: one terminal
uv run swarm sim        # copters, router, watcher, 3D viewer; fly it with `uv run swarm pilot`
uv run swarm rehearse   # the whole mission hands-free (below)
uv run swarm plot logs/watch-<timestamp>.jsonl   # trajectory figure of a flight
```

Where the copters start, A and B, the number of copters, the wind and the rest come from
`mission.toml` (Settings, below). SITL runs headless; each copter logs to `logs/sitl/<i>/`,
the router to `logs/router.log`. QGroundControl is off unless you add `--qgc`: a backup
display and control, nothing in the stack needs it.

## Settings

`mission.toml` holds the mission's settings, each commented. Edit it and restart the command:
each reads it once, as it starts.

- `[mission]`, the sim and the field alike: `point_a` and `point_b` (`[latitude,
  longitude]`) and `copters` (system ids 1 up to it).
- `[swarm]`, the sim and the field alike, the numbers to tune on the day:
  `follower_distance_m` and `follower_height_step_m` shape the formation;
  `liftoff_max_leader_distance_m` and `liftoff_min_leader_height_m` are how close and how high the
  leader must be before a follower lifts off (the first is the viewer's control area ring);
  `follow_max_distance_m` is where a follower stops following and holds (`FOLL_DIST_MAX`,
  more than `liftoff_max_leader_distance_m`); `landing_zone_radius_m` is the landing ring round B.
- `[fence]`, the sim and the field alike: the geofence (below).
- `[sim]`, the sim only: `home` (copter 1's start) and its `ground_alt_m` (AMSL; `"auto"`
  looks it up online when the sim starts and remembers it in `logs/elevation.json`), the
  `spawn_spacing_m` between the copters in their line to the east, the `router` (`"sitl"`,
  or `"field"` to run through `router/field.conf` with each copter sending over UDP like a
  drone's Wi-Fi bridge, see Run on hardware), the wind (`wind_speed_ms`,
  `wind_from_deg`, `wind_turbulence_ms`), and the camera feed's video transmitter power
  (`vtx_mw`, below). Real drones start wherever they stand.

Every key must be there, and a misspelt one stops the command with its name. Keep a file per
flying field and pick it with `--settings PATH`. Flags override the file for one run:
`--point-a LAT,LON`, `--point-b LAT,LON`, `--count`, `--follower-distance`, `--router` and
`--home LAT,LON[,ALT]` (A and B then keep their offsets from home, unless given too).

### Geofence

`[fence]` is a polygon the drones must stay inside (`corners`, `[latitude, longitude]` in
order round it, 3 or more) and a ceiling above the takeoff point (`max_height_m`).
`swarm pilot` (also `fly`, `rehearse`) uploads it to every drone with the MAVLink mission
protocol (`swarm.fence`), reads it back, sets `FENCE_TYPE` (polygon and ceiling),
`FENCE_ACTION` and `FENCE_ALT_MAX`, and then turns `FENCE_ENABLE` on. It does so again for
any drone whose fence differs, so an edit reaches every drone on the next start. A drone
does not arm, and a follower does not lift off, until its fence is on board; the 3D viewer
draws it from the drones.

Each drone then checks the fence itself, 25 times a second, with or without the laptop:

- **Outside, it does not arm** (ArduPilot's pre-arm check), and it needs a position fix to
  arm at all.
- **At the fence it does `action`**: `"brake"` stops it and it hovers in BRAKE; `"rtl"` flies
  it home; `"land"` lands it where it is; `"report"` only warns. GUIDED does not take a
  target outside the fence at all.
- **Back in after a brake:** any mode change turns the fence off for 10 s (ArduPilot's
  manual recovery). The pilot's drone: flick the mode switch low and fly in. A follower stays
  in BRAKE until the next rotary phase gives it its job again.
- `enabled = false` turns `FENCE_ENABLE` off on every drone and leaves the polygon on them.

The settings check that A, B and the sim's start line are inside the fence and that no two
edges cross. The upload needs MAVLink 2 to the drone (ArduPilot's default); over MAVLink 1 the
link refuses to upload, as the drone would take the corners as a mission.

## The mission

One mission, flown two ways: by a real pilot with the radio and the switch panel (`swarm fly`
in the sim, `swarm pilot` on demo day), or hands-free with a script standing in for the pilot,
the radio and the panel (`swarm rehearse`, below). Everything behind the pilot is the same
code. The 3D viewer marks point A and draws point B with its 25 m landing ring.

The rotary is turned one way through the mission, START, BOTH, L, and on to R or OFF if
needed.

1. **Launch.** Rotary OFF, gear UP, ALT arms the leader, BEACON gives it the radio; the pilot
   takes it up. The follower switches arm the followers, which wait on the ground; START lifts
   them off, BOTH flies the square.
2. **To A.** The pilot flies the leader to point A; the square follows it in ArduPilot FOLLOW.
3. **Hold at A.** Rotary L: the followers hold where they are, in GUIDED.
4. **One by one to B.** BEACON off, COWL (auto select) on gives the radio the leader: the pilot
   flies it to B and lands it there. Once it has disarmed, the radio goes to the next drone in
   the air (copter 2, then 3, 4, 5); the pilot flies each to B and lands it.
5. **Done.** Gear DOWN locks the panel.

A and B are `point_a` and `point_b` in `mission.toml` (Settings): in the sim, A is about
330 m WNW of home and B 330 m beyond it.

## Rehearse the mission

```bash
uv run swarm rehearse          # about 10 minutes; exit status 1 on the first step that fails
uv run swarm rehearse --stay   # keep the stack up afterwards to look round the 3D viewer
```

The whole mission, hands-free, on the sim stack, with the 3D viewer up at
http://localhost:8600/ to watch it. A scripted switch panel and radio stand in for the
hardware; everything behind them is the code that flies with the real ones (`swarm.console`,
the launcher and panel rules, the radio link and its failsafe, the watcher feeding the COP).
Each step does what the pilot would, then waits for what ArduPilot and the COP report:

1. Five copters ready, each controller linked; gear DOWN locks, gear UP goes live.
2. ALT arms the leader in GUIDED; BEACON gives it the radio in LOITER; throttle up to 15 m.
3. Copter 5's controller off: its RC pre-arm check fails and its switch arms nothing; on
   again, it arms. Follower switches arm the followers; START lifts them off to hold; BOTH
   flies the square.
4. The leader flies 15 s forward, the square keeping its slots (within 3 m), and yaws 90°:
   the square stays put, north-up (within 3 m).
5. START pauses (followers hold while the leader flies off). The pilot climbs the leader to
   copter 2's height and flies straight at it: copter 2 escapes (AVOID_ADSB) and holds again, the leader is never
   taken over. BOTH rejoins.
6. Copter 4's controller dies in flight: radio failsafe, it keeps following.
7. NAV hands the radio to copter 2: the leader holds in GUIDED on its idle controller;
   copter 2 flies LOITER, STABILIZE, LOITER on the sticks; BEACON back and copter 2 rejoins.
8. PITOT HEAT OFF lands copter 5, ON again calls the landing off. The pilot's controller dies
   flying the leader: it brakes, the square holds on it; link back, a flick of the mode switch
   takes it again. R sends the followers home while the radio keeps the leader; BEACON off and
   the leader goes home too; BOTH calls that off; COWL auto select gives the radio the leader.
   OFF starts the followers landing; BOTH before touchdown calls it off and they rejoin.
9. The mission: the pilot flies the square to point A; L holds it there; then auto select hands
   the radio to each drone in turn, which the pilot flies to its own spot 8 m from B, lands
   (throttle down) and disarms (throttle down, yaw left). Every drone ends within 25 m of B;
   gear DOWN locks the panel and the COP agrees.
10. Stick arming: throttle down with yaw right arms the leader; a hop to 5 m, and gear DOWN
    lands it and locks the panel.

It prints a PASS/FAIL line per step, and on a failure what was missing and each copter's mode,
height and launcher stage. The telemetry log (`logs/watch-<timestamp>.jsonl`) replays in the
viewer with `swarm view --replay`.

## Fly the sim with the radio

```bash
uv run swarm fly   # copters, router, watcher, 3D viewer, radio, switch panel, keys; Ctrl-C stops all
```

`swarm fly` is `swarm sim` and `swarm pilot` in one terminal. Plug in the radio and the switch
panel (on WSL attach them with usbipd) before or after; both are picked up whenever they
appear. The switch panel is drawn at the foot of that terminal, worked with the keyboard
(Switch panel in the terminal, below). To run the parts separately,
start `swarm sim`, then `swarm pilot` in a second terminal.

Watch it in the 3D viewer (http://localhost:8600/). `swarm pilot` reads the USB radio
(`/dev/input/by-id`, attached with usbipd on WSL) and streams its 8 channels at 50 Hz straight
into the SITL receiver input of the copter it flies (copter 1, UDP 5501, until the switch
panel selects another), as a receiver would. No ground station is in the loop. Arm with
throttle down and yaw right. If the radio drops out, its copter gets centred sticks.

The flight-mode switch (channel 6) is read by `swarm pilot`, not ArduPilot: it turns
ArduPilot's own off on every copter (`FLTMODE_CH 0`) and sets the mode itself. Low is
LOITER, middle STABILIZE, high ACRO. A copter the radio gets goes to LOITER, and the switch
acts on it only once it has been low since: going to a new copter, put the switch low,
settle the throttle, then pick the mode.

The followers start disarmed on the ground. Key `a`, `b`, `c` or `d` (no Enter) launches
copter 2, 3, 4 or 5:

1. it arms in GUIDED (and re-arms if ArduPilot disarms it for sitting idle);
2. 5 s after arming, once copter 1 is in the air above 5 m and within 100 m of it
   (`liftoff_min_leader_height_m`, `liftoff_max_leader_distance_m`)
   horizontally, it takes off to the leader's height plus its step;
3. it switches to ArduPilot FOLLOW in its formation slot round the leader (see
   Architecture): 5 m north, east, south and west of it for copters 2 to 5, 1.5, 3, 4.5 and
   6 m above it; one or two in frame on the leader's FPV camera whichever way
   it faces. The 3D viewer's FPV view (`V`) is level (Tilt 0°) by default, FOV 120°.

The copters spawn 5 m apart to the east (`spawn_spacing_m`), all within 100 m of copter 1, so every follower can go
while copter 1 stays near its takeoff spot; fly it well away and they wait for it. Key `l` lands every follower where it is; land them before the leader, since FOLLOW keeps
them at their step above it wherever it goes.
`swarm pilot` uses the router's spare endpoint (UDP 14570); run one tool there at a time.

### Switch panel

A Logitech (Saitek) Pro Flight Switch Panel (06a3:0d67) runs the mission from `swarm pilot`.
On WSL attach it like the radio (`usbipd bind` once from an admin shell, then
`usbipd attach --wsl --busid <id>`) and let your user open it:

```bash
printf '%s\n' \
  'SUBSYSTEMS=="usb", ATTRS{idVendor}=="06a3", ATTRS{idProduct}=="0d67", MODE="0666"' \
  'SUBSYSTEM=="input", ATTRS{idVendor}=="06a3", ATTRS{idProduct}=="0d67", ENV{LIBINPUT_IGNORE_DEVICE}="1"' \
  | sudo tee /etc/udev/rules.d/99-saitek-switch-panel.rules
sudo udevadm control --reload   # then unplug and replug the panel
uv run swarm panel   # prints each switch change; gear LEDs nose green, left red, right amber
```

The second line keeps the desktop off the panel. Linux also reads it as an input device whose
last four contacts land on mouse buttons: rotary BOTH is a left click, START a right click,
gear UP and DOWN middle and back, held for as long as they rest there. `swarm` reads the panel
through hidraw, so it does not need that device.

`swarm pilot` picks the panel up whenever it appears; from then on it is in charge
(`--no-panel` ignores it). Keys `a`-`d` and `l` still work while the panel is live, with
`--no-terminal-panel`; the terminal's panel takes those keys (below).

| Control | Does |
|---|---|
| Gear lever | The safety switch. **DOWN**: every armed drone lands (LAND), the radio's included, the radio flies nothing, and the panel locks. **UP** unlocks (live), but only once the lever has been seen DOWN, so a lever already UP at start arms nothing. |
| ALT | Copter 1, the leader: ON arms it in GUIDED, ready, holding on the ground (and re-armed there) until the radio takes it. OFF lands it; ON again calls the landing off. |
| AVIONICS, FUEL PUMP, DE-ICE, PITOT HEAT | Copters 2-5, as keys `a`-`d`: ON arms in GUIDED and puts it on the mission. OFF lands it where it is; ON again calls the landing off and it takes up the mission again (on the ground, it waits armed to lift off). |
| Rotary START | Launch and hold: armed followers lift off (the key flow above) and hold where they are in GUIDED. From BOTH it is a **pause**, as L. |
| Rotary BOTH | Follow: followers lift off if they have not, and fly their formation slots in FOLLOW. From START or L they rejoin the leader wherever it is. |
| Rotary L | Hold (loiter) at A: the followers stop following and hold where they are in GUIDED, while the radio flies the drones on to B. As START: armed followers still on the ground lift off. |
| Rotary R | Every drone in the air returns home (RTL), the leader included, except the one the radio flies: it goes home as soon as you let go of it (its selector off). Nothing arms. Back to START, BOTH or L while still in the air and they hold or rejoin; the leader holds. |
| Rotary OFF | Land: every drone in the air lands, the leader included, except the one the radio flies: it lands as soon as you let go of it. Drones armed on the ground (switch ON) wait there, so OFF is also where you arm before START. Turn to START, BOTH, L or R while they are still in the air and the landing is called off. |
| BEACON | Radio selector: the radio flies the leader. |
| NAV, STROBE, TAXI, LANDING | Radio selector: the radio flies copter 2-5 (followers `a`-`d`). |
| COWL | Auto select: the radio flies the leader, then, whenever the drone it flies is disarmed, the next drone in the air (leader, `a`, `b`, `c`, `d`, round). |
| BAT, PANEL | Nothing. |

**Radio selector.** The bottom row (COWL to LANDING, PANEL aside) picks the drone the radio
flies; the leader too needs BEACON (or COWL) on. Exactly one may be on: two or more select
nothing until all are off again, then turn one on. With none on, the radio flies nothing:
the leader's job is then to hold where it is (GUIDED), and the followers follow it or hold, by
phase. The drone selected goes to LOITER under the sticks at once and the phases leave it
alone; the mode switch picks its mode once it has been low (above). Deselected, a drone goes
back to the autopilot: a follower takes up the current phase's job (FOLLOW, hold, RTL or LAND)
and is held there for 5 s through the RC failsafe of losing the radio; the leader holds
where it is. Landing the radio's drone (its switch, `l`) keeps the radio on it, but the mode
switch waits for low again, so a flick of it does not undo the landing.

In the sim the radio is moved by `swarm pilot`, which streams it to the selected copter's
SITL RC port; the copter left behind gets its own controller back, idle (see Pilot override).
On hardware each drone has its own controller, so the pilot picks up the controller of the
drone the panel selects.

A rotary position counts once the rotary has rested there for 0.75 s, so turning it past L
and R on the way to OFF neither holds nor sends anything home. Everything but the gear acts
only while live. A switch already ON when the panel goes live acts then. A drone that has
landed and disarmed, by its switch, OFF or gear DOWN, flies again once its switch is cycled.
Gear indicators: nose red locked, amber live with the radio flying nothing (none selected,
two selected, or the mode switch not yet low), green live with the radio flying a drone; left
the leader and right the followers, amber arming or on the ground, green flying, red landing
or returning home.

### Switch panel in the terminal

```bash
uv run swarm panel-demo   # the panel in the terminal flying pretend drones: no sim, no radio
```

`swarm pilot` and `swarm fly` also draw the switch panel at the foot of the terminal; the logs
scroll above it. It has every control of the hardware, where it sits on the panel, labelled
with the first two letters of its name there (BOTH is BO, AVIONICS AV, FUEL PUMP FU; the gear
lever's GE UP and GE DO), with the drone each switch acts on under it and the gear LEDs lit as
on the hardware. The top edge shows the nose LED's state: LOCKED, or LIVE with the radio idle or
flying.

| Key | Does |
|---|---|
| Arrow keys | Move to the nearest control that way. |
| Enter or space | Flip the switch, turn the rotary to the position, or move the gear lever there. |
| `1`-`5` | Flip an arming switch: ALT (the leader), AVIONICS, FUEL PUMP, DE-ICE, PITOT HEAT. |
| `6`-`9`, `0` | Flip a radio selector: BEACON (the leader), NAV, STROBE, TAXI, LANDING. |
| `A` | Flip COWL, auto select. |
| `O` `R` `L` `B` `S` | Turn the rotary to OFF, R, L, BOTH, START. |
| `U` `D` | Gear lever UP, DOWN. |

Letters work in either case, and a shortcut moves the cursor to its control. The panel takes
every key: the launcher's keys `a`-`d` and `l` work only with `--no-terminal-panel`.

The same rules act on it as on the hardware: it is one more switch panel. It starts with the
gear lever between its positions, so it goes live only after GE DO then GE UP. With the
hardware attached the two work together: the terminal takes the hardware's positions when it
first finds it (unless something was already moved in the terminal), then a change on either
moves the switch. A `*` marks a control where the hardware is set differently, so a switch
moved in the terminal stays as it is until moved on the hardware. `--no-terminal-panel` turns
it off; `--no-panel` turns off both. A terminal under 14 rows shows no panel.

`swarm panel-demo` runs the real launcher and panel rules on pretend drones: each arms when
told, climbs on a takeoff, holds, follows its leader's height, and comes down in LAND or RTL.
The radio is a pretend pilot that flies the armed drone it is given up to 12 m. Flown through
the mission (`D`, `U`, `S`, `1`, `6`, `2`-`5`, `B`, `L`, `O`, `6` again) the logs and
LEDs show what each switch does. The hardware panel works in it too when attached.
`swarm rehearse` draws the panel as its script flips the switches.

## 3D viewer

`uv run swarm sim` also starts the 3D viewer: open http://localhost:8600/. It is a CesiumJS
page fed by the watcher's telemetry (UDP 14600), so it shows exactly what the drones report,
live, in sim and on hardware.

```bash
uv run swarm sim                       # the viewer starts with the stack; --no-view skips it
uv run swarm watch                     # hardware: telemetry for the viewer, commanding no drone
uv run swarm view                      # hardware: the viewer, fed by `swarm watch`
uv run swarm view --replay logs/watch-<timestamp>.jsonl --speed 2 --loop
```

`--mark NAME=LAT,LON` adds map markers.
`--view-host 0.0.0.0` lets a tablet on the same network open it.

- **Drones**: 3D models with attitude, MIL-STD-2525E symbols (friendly military unmanned
  aircraft, CoT type `a-f-A-M-F-Q` as ATAK draws it), trails, altitude stalks, and role, mode,
  height and speed for the leader and the selected drone. Each drone has a colour, blue until
  you click its fleet card's colour bar and pick another (black, white, red, blue, green,
  yellow, orange, purple, pink, grey); the browser remembers it. Its trail, sensor, mission
  and selection outline are drawn in it (ALL or `0` selects every drone). Who is flying shows
  on the card and in FPV as 2525E's common modifiers: `RP` (remotely piloted: armed in a pilot
  mode, STABILIZE, ALT_HOLD, LOITER, POSHOLD, ACRO, SPORT, DRIFT) or `AUT` (autonomous: the
  autopilot). The map symbol carries `AUT` only, as the two are hard to tell apart at that
  size. The propellers show the state: still when disarmed, a slow turn when armed on the
  ground, faster in the air.
- **Control area** (`K`): a ring on the ground 100 m (`liftoff_max_leader_distance_m`) round the
  leader. An armed follower inside it lifts off (`swarm pilot`).
- **Swarm**: formation slots and the ring round the leader, the park line at A, point B with its
  25 m landing radius and its track, range and bearing from the selected drone to B.
- **Plans** (`M` mission, `G` fence): whatever is uploaded to each drone, from QGroundControl
  or anything else. The watcher reads each drone's mission, geofence
  and rally points over MAVLink (`swarm.mission`; read only, refreshed when the mission count
  changes and every 10 s). Drawn ATAK-style: a solid route in the drone's colour, points
  named SP (takeoff), CP1, CP2… and LZ (land) with their heights, drop lines to the ground,
  QGC's item letters (T, L, H), the current point green, flown legs faded, RTL climbing to
  `RTL_ALT_M`. The selected drone's fence: the `FENCE_RADIUS` circle around home up to
  `FENCE_ALT_MAX`, inclusion and exclusion zones, breach return point, dashed while the fence
  is off.
- **Sensors**: the camera's view (selected drone, all, or off; tilt and FOV sliders): a line
  from the drone along each image edge, fading out 60 m ahead, with the image frame across
  them. **FPV** looks through the same camera, with a HUD: what it shows lies between the
  lines.
- **Camera**: **Auto**, which chases the leader while the swarm moves and orbits it while it
  holds, for hands-free demos; Overview, Top, Follow (chase the selected drone) and FPV, with
  the camera's tilt and FOV sliders, in one bar after the display options; then **Camera feed**
  in a bar of its own. **FPV** (`V`) is the COP's own: the map, symbols and all, seen from the
  drone on its telemetry. **Camera feed** (`C`) is what the drone's camera itself sends: in the
  field its video; in the sim (`swarm sim`, `fly`, `rehearse`), SITL's physics
  (`swarm.physics`), streamed as each state arrives with no buffer. It shows only real things,
  every drone at its real size where its physics puts it and the truck at B: no symbols, labels,
  trails, slots or plans, cropped to an analog NTSC camera's 4:3 frame (`CAMERA_ASPECT` in
  `app.js`) with black round it, as goggles show it. In the sim it comes over a simulated
  analog video link (`analog.js` from
  [analog-vision](https://github.com/Synetic-Labs/Analog-Vision), its chain on the GPU,
  vendored by `scripts/setup.sh`): the picture is encoded as
  NTSC composite, frequency-modulated as a 5.8 GHz video transmitter does it, given the
  receiver's noise, and decoded as goggles do, sync, AGC, colour killer and all. The noise
  comes from a link budget: `[sim] vtx_mw` over free space from the drone to the takeoff point,
  where the pilot stands, a dipole upright on each, with slow fading on top. So as the drone
  flies away the picture gets grainy, then the colour speckles, sparklies come, the colour
  drops out, lines tear and it snows; flying overhead, or banked hard toward the pilot, puts it
  in the antennas' null. Without WebGL2 the picture is shown clean. In it, ArduPilot's own OSD
  as the drone's analog video carries it, laid in by the flight controller's OSD chip (MAX7456,
  `OSD_TYPE 1`): 30 characters by 16 rows, of which NTSC shows the top 13. SITL has no OSD
  chip, so it sends the same screen over MSP DisplayPort (`OSD_TYPE 5`, `SERIAL5_PROTOCOL 42`
  in `sitl/fpv5.parm`), 10 screens a second, to UDP 14700 + sysid (`swarm.osd`), and the page
  puts its top 13 rows into the video across the frame in the chip's font (`OSD_FONT`, from
  `external/ardupilot`), white and black in place of the picture as the chip does, so the
  link spoils it too. Its layout is the drone's `OSD1_*` parameters, so set it up in the sim
  as on the drone: an item on rows 13 to 15 (ArduPilot puts ground speed and the GPS position
  there) does not show on NTSC. Without physics or the OSD (a replay) it falls back to telemetry and a HUD,
  and says `NO FEED`.
- **Readouts**: a fleet card per drone: its callsign with the mode as a light beside it
  (amber: a pilot flies it with no position hold, STABILIZE, ACRO, ALT_HOLD; green: a pilot
  with GPS hold, LOITER, POSHOLD; cyan: the autopilot, AUTO, GUIDED, FOLLOW; red: returning,
  landing, braking or avoiding; off while disarmed) and its role, five dials side by side, and
  the battery: remaining as a bar, and the voltage ArduPilot measures (SYS_STATUS). Watching
  (`swarm fly`), the leader is the drone the followers follow (`FOLL_SYSID`), LEADER (1);
  a copter the radio takes over instead is PILOT. The dials are SkyFlow style: an aircraft's
  counter-drum-pointer altimeter (the needle turns once every 100 m over 0-9, a drum counter
  under it rolls the metres), an aircraft-style vertical speed dial (VS: level at 9 o'clock,
  needle up to climb, ±5 m/s),
  ground speed (GS, 0-20 m/s), compass and attitude (horizon line, yellow aircraft symbol).
  Every number is the autopilot's own estimate over MAVLink: altitude from its EKF (mostly
  the barometer), speeds from GPS and accelerometers fused, heading from gyros and compass,
  attitude from the IMU; none of it raw GPS.
  Round them, as QGC draws them, the sticks from RC_CHANNELS: throttle left of the altimeter,
  yaw above the compass, roll above and pitch right of the attitude dial (dimmed while the
  autopilot flies). Glass-cockpit colours on black: white scales, green values, cyan stick
  input. Hover a dial for its units. Speed, heading and attitude live only there (and in the
  FPV HUD; the panel, compass and toasts stay up in FPV). Cursor position in MGRS, toasts for leader changes, landings, mode changes and phases.

Keys: `1`-`5` select, `0` select all, `O` overview, `T` top, `F` follow, `V` FPV, `C` camera
feed, `A` auto, `S` sensors,
`L` labels, `R` range and bearing, `M` mission, `G` fence, `K` control area,
`Esc` free camera.

Mouse, as in Google Earth and Mapbox: left drag pans, right or middle drag (or `Ctrl` + left
drag) orbits the point under the cursor, wheel or trackpad pinch zooms, `Shift` + left drag
looks around. Click a drone to select it, double-click it to follow it, double-click the
ground to fly in toward that point.

The map needs internet. Without keys it uses Esri World Imagery on Esri World Elevation.
Keys go in `.env.local` (git-ignored) or the environment:

| Key | Map |
|---|---|
| `GOOGLE_MAPS_API_KEY` | Google Photorealistic 3D Tiles (Map Tiles API) |
| `CESIUM_ION_TOKEN` | Cesium World Terrain, Bing imagery, OSM buildings |

`scripts/setup.sh` puts CesiumJS, milsymbol, mgrs and Rapier in `external/web/`. No Node.js is
needed.

The drone is Sketchfab's [FPV Drone](https://sketchfab.com/3d-models/fpv-drone-92152c31ec9a4a87b2a0478bd114d293)
by PinaPipe (CC BY 4.0), committed as downloaded in `assets/fpv_drone.glb` (Sketchfab needs a
login). `swarm.fpvmodel` converts it into the model the COP serves,
`src/swarm/web/models/fpv-drone.glb`: in metres, nose forward, textures at 512 px, propellers
spinning ArduPilot's quad X way while armed. After changing the converter:

```bash
uv run python -m swarm.fpvmodel assets/fpv_drone.glb src/swarm/web/models/fpv-drone.glb
```

A truck is parked on point B, facing north: Sketchfab's
[Military Truck](https://sketchfab.com/3d-models/military-truck-d3a85af5faef4fddb217b3fce073614c)
by Raphael Escamilla (CC BY 4.0), committed as downloaded in `assets/military_truck.glb`.
`swarm.truckmodel` converts it into `src/swarm/web/models/military-truck.glb`: in metres (4.9 m
long), front forward, one mesh per material, textures at 512 px, and only the glass
see-through. A drone that strikes the truck, coming within `HIT_MARGIN_M` (`fleet.js`) of it,
puffs white smoke there (`smoke.js`). After changing the converter:

```bash
uv run python -m swarm.truckmodel assets/military_truck.glb src/swarm/web/models/military-truck.glb
```

### Drive (demo)

Drive the truck on the Google 3D tiles, with physics and collisions. It needs
`GOOGLE_MAPS_API_KEY`.

```bash
uv run swarm drive                             # starts at point A; open the URL it prints
uv run swarm drive --at 45.319749,-75.758425   # starts here
```

Or open http://localhost:8600/drive.html on a viewer that is already running (`swarm sim`,
`swarm view`). `?at=LAT,LON,HEADING` sets the start, the heading in degrees true.

Keys: `W` `A` `S` `D` or the arrows drive and steer (`S` brakes, then reverses), `Space`
handbrake, `R` puts the truck back on its wheels, `C` cameras (chase, far, hood, free), `G` shows
the colliders, `M` turns the sound on and off, `H` hides the help. A gamepad works: left stick steers, the right trigger
accelerates, the left brakes. So does the drone radio in joystick mode, with a panel of its own
(`driveradio.js`): the throttle, its full travel, drives in the gear a switch picks (forward,
neutral, reverse, which brakes first), a stick steers, a switch is the handbrake and a flick of
another resets the truck. The panel shows each live, and every channel; **Set** on a row, then
moving that stick or switch where the panel asks, picks its channel and which way round it is,
kept in the browser. Out of the box: throttle CH3, steer CH4 (yaw), gear CH6, handbrake CH5,
reset CH7. The page gets the radio from the viewer (`/radio`), which reads it from Linux like
`swarm pilot` does (`swarm.radio.RadioReader`): Chrome's Gamepad API on Linux misses most of
a drone radio's axes. The throttle stick stays where it is left, so it drives nothing until it has been at
the bottom once each time the truck is placed. Double-click the map to move the truck there.

[Rapier](https://rapier.rs) (WebAssembly) runs the truck as a ray-cast vehicle: a 2.2 t box
chassis on four sprung wheel rays, in an east-north-up frame round the start. The ground is the
map itself. `tilecolliders.js` reads back from the GPU the triangles of each tile Cesium draws
within 150 m of the truck, about 2 ms a tile, and makes each tile a Rapier trimesh. The
colliders change level of detail with the map, so the truck drives on and hits exactly what is
drawn: kerbs, walls, parked cars, and bushes and trees too, solid as the photogrammetry has
them. Physics takes under 0.2 ms a frame. Tiles coarser than 8 m are never built. The page
turns off Cesium's own camera collision (`enableCollision`): it reads triangles back under the
camera every frame the camera moves, a tenth of the frame time. The chase camera stays out of
walls with a ray cast on the colliders instead.

### Wing (demo)

Fly the TBS Chupito, an 800 mm pusher flying wing, on the map. A first, basic demo: its only
obstacle is the ground (the 3D tiles' roofs included).

```bash
uv run swarm wing                              # starts at point A; open the URL it prints
uv run swarm wing --at 45.319749,-75.758425    # starts here
```

Or open http://localhost:8600/wing.html on a viewer that is already running.

Keys: `Space` throws it (and again once it is down), the arrows pitch and roll (`↓` pulls the nose
up), `W` `S` throttle, `R` back to the start, `C` cameras (chase, FPV, free), `H` hides the help.
A gamepad flies it on the right stick, the right trigger the throttle, A throws it. So does the
drone radio in joystick mode, as mode 2: raising its throttle from the bottom throws it.

The flight model (`wingflight.js`) is Nanawing's, narenana's browser sim
(https://sim.narenana.com/), ported as it is from its deployed bundle. Its source is not public.
It is an arcade model, not a rigid body: the sticks set the body's rates directly, and lift,
drag, thrust and gravity move it. The model is narenana's recreation of the Chupito.
`scripts/setup.sh` fetches it into `external/web/chupito/` rather than committing it, because it
has no licence and carries Team BlackSheep's logos.

### Wing on INAV SITL

The real Chupito flies INAV (TBS's Lucid H7 Wing AIO, target `TBS_LUCID_H7_WING_MINI`), so the
simulated one does too. INAV SITL has no physics of its own; it flies whatever answers it over
X-Plane's UDP protocol. `swarm.wingsim` answers it with a flight model of our own.

```bash
uv run swarm wingsim --demo     # no radio: arms, throws it, circles
uv run swarm wingsim            # the drone radio on INAV's sticks
```

Watch it at http://localhost:8600/wing.html, on the map's 3D tiles. The page only draws it: the
flight runs in Python. The page reports the map's ground under the wing, roofs and trees
included, and the flight model flies over that ground. On the page, Space throws it, R puts it
back in the hand, and C cycles the camera: chase, far chase, FPV and free. `--no-view` leaves the
viewer out, and `--view-port` moves it (the Google tiles' key only works on 8600).

The FPV camera shows INAV's own OSD, as on the real wing. TBS sets INAV up for a DJI air unit:
it sends its OSD over MSP DisplayPort on UART3, in Betaflight-compatible HD, a 53 x 20 grid of
Betaflight's glyphs, which the goggles draw. The sim does the same with TBS's OSD layout.
`swarm.wingrun`'s `AirUnit` polls INAV as the air unit does, since INAV draws nothing until
something does. The page draws INAV's screen in Betaflight's font, which `scripts/setup.sh`
fetches. INAV's symbols that Betaflight's font lacks show as `?`, as they do in DJI's goggles.
The goggles' own status bar is not drawn. The battery reads 0 V: INAV 9.1 SITL takes a battery
only from its own X-Plane plugin, which the sim does not play.

The page shows the flight model at work. D opens the panels of numbers. Keys 1-5 draw layers on
the wing, any of them together, with or without the panels:
1. **Forces:** each strip's force, coloured by how attached its flow is, plus the prop, the pod's
   drag, gravity and the totals.
2. **Air:** the airflow, the air past each strip, the wind and the trailing vortices.
3. **Lift & balance:** each strip's patch of the wing tinted by its flow, and the centre of mass,
   centre of pressure and neutral point on the root chord.
4. **Rotation:** the net moment and the rotation rate about each axis.
5. **Path:** a 30 s trail. Shift+5 switches its colour between load factor, airspeed, attached
   flow and energy gain.

[ and ] size the arrows.

Keys in the terminal, then Enter: `t` throws it, `r` puts it back in the hand, `q` quits.
Each run is recorded to `logs/wingsim/` (CSV, 50 Hz): the wing's true state, the sticks, INAV's
outputs, its own estimate of the attitude, and the wind.

The wind is mission.toml's (`wind_speed_ms`, `wind_from_deg`, and `wind_turbulence_ms` for
gusts), or `--wind`, `--wind-from` and `--gusts` for one run:

```bash
uv run swarm wingsim --wind 8 --wind-from 270 --gusts 5
```

INAV's modes:
- **CH5 high** arms.
- **CH6** low is ANGLE, middle ACRO, high MANUAL.
- **CH7 high** is RTH.
- **CH8 flicked high** throws it. This one is the sim's, not INAV's.

INAV's launch mode is on, as TBS sets it: arm, raise the throttle, throw, and INAV runs the
motor and climbs out. INAV's settings are TBS's Chupito CLI suggestions, plus the simulator's
sensors and receiver (`swarm.wingsim.inav`). Its eeprom and log are in `external/inav/chupito/`.
`scripts/setup.sh` puts the SITL binary in `external/inav/`. It is the one INAV's Configurator
ships, because building INAV needs Ruby.

The flight model is a 6-DOF rigid body:
- **Aerodynamics over the full envelope:** Selig's method for aerobatic model aircraft, the
  way X-Plane does it too. The wing is cut into strips. Each strip sees the air at its own
  place, including the aircraft's rotation, less the downwash a vortex lattice of the measured
  planform puts there. Each strip stalls on its own, with section data over ±180°. So stalls,
  wing drop, spins, tail slides, reversed flow and elevons fading in separated flow come out of
  the geometry; nothing is scripted. The fin is a strip, and the pod adds its drag and Munk
  moment.
- **Stall memory:** each strip's flow takes time to separate and to reattach (Goman and
  Khrabrov). A quick pull outruns the stall for a moment (dynamic stall). Once stalled, the flow
  comes back only a couple of degrees below the stall angle (hysteresis).
- **Ground effect:** near the ground, the downwash weakens strip by strip at each strip's own
  height (McCormick). The wing floats in the flare, and a low wing tip is pushed back up.
- **Wind:** MIL-F-8785C's low-altitude model, as JSBSim has it. The wind falls off toward the
  ground (the log profile). Dryden turbulence matches it in strength, including how it varies
  across the wing, which rolls, pitches and yaws the wing. Gusts come on top, now and then.
- **Power train:** the motor, the battery, and a prop with UIUC's measured 5x3 shape. It
  windmills, loads up in reverse flow, kicks the airframe when the throttle is punched, and acts
  as a fin behind the centre of mass. The spinning rotor adds its gyroscopic moment.
- **Ground:** a hand launch and ground contact.

It is calibrated to TBS's specification and to one pilot's INAV blackbox log of a 6S Chupito:
- **Speed:** full throttle gives 42 m/s level on 540 W, and 35 % throttle cruises at 58 km/h.
- **Stall:** 35 km/h with the elevons holding it.
- **Roll:** full elevons roll at 600-700°/s.

Checked against the log without fitting to it: full pitch command pulls about 7 g at 29 m/s,
and a pitch release at the pilot's speed overshoots the way the real wing's did.
`swarm/wingsim/airframe.py` gives the source of every number, and `tests/test_wingsim.py` holds
the model to them.

The package imports nothing else from `swarm`, so it can move to a package of its own. The glue
lives outside it, in `swarm/wingrun.py`: the radio, the demo pilot and the status line.

### Duel

A match between two players, each on their own machine with their own sim and COP. In each
turn one player is the pilot: they fly the mission with their five drones and strike the other
player's truck. The other drives the truck, in their COP, and keeps away. Then they swap. Both
need `GOOGLE_MAPS_API_KEY`. The two COPs meet browser to browser over WebRTC
([Trystero](https://github.com/dmotz/trystero), fetched by `scripts/setup.sh`). Public Nostr
relays set the connection up; after that no server sits between them.

1. Each player runs `uv run swarm fly` (or `swarm sim` and `swarm pilot`) and opens the COP.
2. One presses **Duel** (`D`), types a name and opens a room. They are the host and fly first.
   The other joins with the room's code.
3. The host sets the rounds (up to 5) and the minutes a turn (2 by default), and starts the
   match. In each round both players have a turn as the pilot, the host first.
4. The driver's COP turns into the drive: the truck on the pilot's point B, the pilot's fence as
   a red wall, the drones as they come, each with its range. The pilot sees the truck in place
   of the one parked on B, and the range and bearing line points at it.
5. The pilot starts the turn once their drones are ready, on the ground at home. They fly the
   mission as they like: the swarm, or one drone at a time. The truck goes back to B.
6. Each drone that strikes the truck scores a point. A turn ends when its time is up or the
   pilot has no drone left. The panel keeps the tally, round by round; the match ends after the
   last round, or sooner once the trailing player can no longer catch up, with the final score
   over the map.

A drone is out, its link gone as if destroyed, once it strikes the truck or crashes: at the
ground after it has flown, inside the map's 3D tiles (a tree, a building, a hill, by 1 m or
more), or into another drone. Only LAND puts one down safely; a stick landing, RTL's or AUTO's
counts as a crash. The pilot's COP judges all of it on what its pilot sees, and has the sim
stop that copter's SITL (`POST /sim/destroy`). When a player's turn ends their sim restarts
every copter where it began, its parameters kept (`POST /sim/reset`); a restarted copter needs
about 30 s for its GPS position, and Start turn waits for it. On real drones nothing is
stopped: an out drone only leaves the map.

A truck outside the fence for 5 s goes back to its last place inside. `?room=CODE&as=host` (or
`as=guest`) on the COP's address opens the room on load; the host's page keeps the match
through a reload, and a turn under way when a player leaves starts again. Peers behind strict
NATs may not connect without a TURN server; the pages report it. Every message is in
`src/swarm/web/duel.js`.

### Sound

The pages make their sounds in the browser with Web Audio (`src/swarm/web/sounds.js`), with no
files to fetch. In the duel: a strike (and the score, for the pilot), a drone down, a turn
starting and ending, the last 10 seconds, the match won or lost, the opponent coming and
going. While driving: the truck's engine, following its speed and throttle through four gears,
a bump when it hits something, a warning each second outside the fence, and, in a turn, a ping
that quickens as the nearest drone closes on the truck.

Each drone sounds like a 5 inch quad: its four motors, each from its own output in
SERVO_OUTPUT_RAW (motor speed off ArduPilot's thrust curve), whining at their blade rate a
little out of tune with each other, with the prop wash under them. The props spool up and run
down, a punch out opens the sound up, and a flip pulls the four apart. Each drone is heard
from the map's camera: through a drone's camera, that drone at full loudness; from farther,
quieter, panned left or right, its pitch rising as it closes and dropping as it passes. A
landed, disarmed drone is silent. The nearest 6 within 600 m are heard. The driver in a duel
hears the pilot's drones coming, their motor outputs sent with their poses. A log from before
the motor outputs plays each armed drone at hover.

Each sound is fixed in `sounds.js`, the same for both players. **Sound** in the toolbar, or `M`
while driving, turns all of it on or off; the browser remembers which, and the COP and the
drive page follow a change straight away. Browsers start audio only after a click or a key on
the page, and a hidden page is silent.

## Sim airframe: a 5 inch quad

SITL flies `sitl/fpv5.json`, a typical 5 inch freestyle quad X, with the tune in
`sitl/fpv5.parm` (from ArduPilot's own freestyle model): 750 g all up, 225 mm motor to motor,
6S, 2207 motors on 5.1 inch props at about 1.7 kgf each, **thrust to weight 9**, drag settling
at 22 m/s with a 45° lean. Measured in SITL: hover at 23 % motor output, 10 m in 5.4 s in a
GUIDED takeoff. The pack is unlimited in the sim; a real 6S 1300 mAh gives about 4 to 6
minutes. Edit `hoverThrOut` (1 / thrust to weight), `mass` and the rest in the JSON to try
another build. The mission's speeds stay in `swarm.parm` (`LOIT_SPEED_MS`, `WP_SPD` 10 m/s),
so in ACRO the pilot can outrun the followers.

`fpv5.parm` stands in for each real drone's own tune: do not load it onto real drones.

## Run on hardware

1. Flash ArduCopter 4.7 and apply `sitl/swarm.parm` values (unique `MAV_SYSID`
   per drone). Tune each airframe on the airframe itself (autotune); `sitl/fpv5.parm` is the
   sim's.
2. Point drone N's Wi-Fi MAVLink bridge at the laptop, UDP 14550 + N (drone 1 on 14551 to
   drone 5 on 14555). One port per drone: mavlink-router answers a UDP port only at the last
   address it heard from and never relays a message back to the port it came in on, so drones
   sharing a port miss most commands and never hear each other's positions (FOLLOW and
   avoidance need them). `swarm sim --router field` (also `fly`, `rehearse`) runs the sim
   through `router/field.conf` the same way, each copter sending over UDP like its bridge.
3. `external/mavlink-router/build/src/mavlink-routerd -c router/field.conf`
4. Set `point_a`, `point_b`, the `[fence]` corners (and `copters`) in `mission.toml` for the
   field, or in a copy
   given to each command with `--settings PATH`. `uv run swarm watch` and `uv run swarm view`,
   then open http://localhost:8600/ for the 3D view (QGroundControl optional, as a backup
   display and control). The watcher sends B to the viewer in its telemetry; the viewer
   marks A.
5. `uv run swarm pilot --no-radio`: the switch panel runs the mission. Each drone's own
   controller flies it; the laptop has no radio to read.

## Development

```bash
uv run pytest -q
uv run ruff check src tests && uv run ruff format src tests
uv run basedpyright src tests
```
