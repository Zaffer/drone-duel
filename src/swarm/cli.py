"""Command-line entry points."""

import argparse
import logging
import signal
import threading
import time
from collections.abc import Iterator
from contextlib import contextmanager, nullcontext
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING

from swarm.link import MavlinkLink
from swarm.telemetry import DEFAULT_PORT, TelemetryPublisher
from swarm.watcher import Watcher

if TYPE_CHECKING:
    from swarm.console import PanelDevice
    from swarm.launch import Launcher
    from swarm.settings import Settings
    from swarm.termpanel import PinnedScreen, TerminalPanel
    from swarm.view import Marker, ViewServer

WATCHER_ENDPOINT = "udpout:127.0.0.1:14560"
"""The watcher's endpoint of mavlink-router (router/sitl.conf and router/field.conf)."""
TOOLS_ENDPOINT = "udpout:127.0.0.1:14570"
"""The pilot's endpoint (`swarm pilot`), beside the watcher's."""


def _log_path(name: str) -> Path:
    logs = Path("logs")
    logs.mkdir(exist_ok=True)
    return logs / f"{name}-{datetime.now():%Y%m%d-%H%M%S}.jsonl"


WATCH_LINK_LOG = "swarm.link.watch"


def _build_watcher(conns: list[str], args: argparse.Namespace) -> Watcher:
    link = MavlinkLink(conns, log_name=WATCH_LINK_LOG)
    publisher = TelemetryPublisher(DEFAULT_PORT, _log_path("watch"))
    return Watcher(link, publisher, point_b=_settings(args).field.point_b)


def cmd_watch(args: argparse.Namespace) -> None:
    conns: list[str] = args.conn or [WATCHER_ENDPOINT]
    watcher = _build_watcher(conns, args)
    logging.info("watching %s for the 3D viewer; no vehicle is commanded", conns)
    try:
        watcher.run_until(lambda: False, float("inf"))
    except KeyboardInterrupt:
        pass


def cmd_pilot(args: argparse.Namespace) -> None:
    _run_pilot(args, args.conn or [TOOLS_ENDPOINT])


RADIO_RETRY_S = 1.0


def _run_pilot(args: argparse.Namespace, conns: list[str]) -> None:
    """Radio, switch panel and keys until Ctrl-C. The radio and the panel are picked up
    whenever they are attached."""
    from swarm.console import Console
    from swarm.keys import KeyReader
    from swarm.launch import KEYS
    from swarm.panel import SwitchPanel, find_panel
    from swarm.radio import RadioBridge, find_radio

    settings = _settings(args)
    link = MavlinkLink(conns)
    followers = [s for s in range(1, settings.copters + 1) if s != args.leader][: len(KEYS)]
    launcher = _launcher(link, args.leader, followers, settings, own_mode_switch=not args.no_radio)
    panel = None if args.no_panel else SwitchPanel()
    term, screen = _terminal_panel(panel, launcher, args.no_terminal_panel or args.no_panel)
    console = Console(launcher, term or panel)
    radio: RadioBridge | None = None
    radio_checked_s = -RADIO_RETRY_S

    def find_and_start_radio(now: float) -> None:
        nonlocal radio, radio_checked_s
        if radio is not None or args.no_radio or now - radio_checked_s < RADIO_RETRY_S:
            return
        first = radio_checked_s < 0
        radio_checked_s = now
        device = Path(args.device) if args.device else find_radio()
        if device is None:
            if first:
                logging.warning("no radio in /dev/input/by-id yet; plug it in (usbipd on WSL)")
            return
        radio = RadioBridge(device, launcher.radio_sysid, fleet=[args.leader, *followers])
        radio.start()
        console.radio = radio

    find_and_start_radio(time.monotonic())
    print(f"The radio flies copter {args.leader} until the switch panel selects another.")
    print("Mode switch: low LOITER, middle STABILIZE, high ACRO, once it has been low for that")
    print("copter. Arm on the sticks: throttle down, yaw right.")
    keys = "Its arming switches arm" if term is not None else "Keys a-d arm"
    print(f"{keys} a follower; it lifts off 5 s later once the leader is above")
    print(f"{launcher.leader_min_alt_m:g} m and within {launcher.control_radius_m:g} m of it, then")
    print("flies its formation slot round it in ArduPilot FOLLOW.")
    print("Ctrl-C quits." if term is not None else "l lands every follower. Ctrl-C quits.")
    if panel is not None:
        found = "found" if find_panel() else "not attached yet"
        print(f"Switch panel {found}: gear DOWN then UP to go live; bottom row picks the radio's")
        print("copter; rotary START launches and holds, BOTH follows, L holds, R home, OFF lands.")
    if term is not None:
        print("The switch panel is also at the foot of the terminal: arrow keys and Enter, or")
        print("1-5 arm, 6-0 radio, A auto, O R L B S rotary, U D gear.")
    print("\n".join(launcher.describe()), flush=True)
    period = 0.05
    try:
        with KeyReader() as reader, screen or nullcontext():
            while True:
                tick = time.monotonic()
                find_and_start_radio(tick)
                if term is not None:
                    term.feed(reader.read())
                console.tick("" if term is not None else reader.keys())
                time.sleep(max(0.0, period - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        pass
    finally:
        if radio is not None:
            radio.stop()
        if panel is not None:
            panel.close()


def _terminal_panel(
    inner: "PanelDevice | None", launcher: "Launcher", off: bool, interactive: bool = True
) -> "tuple[TerminalPanel | None, PinnedScreen | None]":
    """The switch panel at the foot of the terminal, merged with `inner`; none off a terminal."""
    from swarm.termpanel import SCRIPTED_HELP, PinnedScreen, TerminalPanel, roles

    if off or not PinnedScreen.usable():
        return None, None
    screen = PinnedScreen()
    followers = {key: f.sysid for key, f in launcher.followers.items()}
    labels = {} if interactive else {"help": SCRIPTED_HELP, "source": "script"}
    term = TerminalPanel(inner, roles(launcher.leader, followers), screen, interactive, **labels)
    return term, screen


def cmd_panel_demo(args: argparse.Namespace) -> None:
    """The switch panel in the terminal flying pretend drones: the real launcher and panel
    rules, no sim and no radio. The hardware panel works too if attached."""
    from swarm.console import Console
    from swarm.keys import KeyReader
    from swarm.panel import SwitchPanel
    from swarm.paneldemo import PILOT_ALT_M, DemoLink, DemoRadio
    from swarm.termpanel import PinnedScreen

    if not PinnedScreen.usable():
        raise SystemExit("panel-demo: run it in a terminal")
    settings = _settings(args)
    home = settings.field.home
    link = DemoLink(settings.copters, (home.origin_lat_deg, home.origin_lon_deg))
    followers = list(range(2, settings.copters + 1))[:4]
    launcher = _launcher(link, 1, followers, settings, own_mode_switch=True)  # pyright: ignore[reportArgumentType]
    hardware = None if args.no_panel else SwitchPanel()
    term, screen = _terminal_panel(hardware, launcher, off=False)
    assert term is not None and screen is not None
    console = Console(launcher, term, DemoRadio(link))
    print(
        f"Switch panel demo: {settings.copters} pretend drones, the real launcher and panel rules."
    )
    print("Arrow keys move, Enter or space flips; or 1-5 arm, 6-0 radio, A auto, O R L B S")
    print("rotary, U D gear. Try: D, U (live); S (rotary START); 1 (arm the leader); 6 (the")
    print(f"radio takes it up to {PILOT_ALT_M:g} m); 2, 3... (followers lift off); B (follow);")
    print("L (hold); O (land); 6 again (the radio lets the leader go: it takes up the phase,")
    print("so at O it lands too). D lands everything.")
    print("Gear LEDs: left the leader, right the followers. Ctrl-C quits.", flush=True)
    try:
        with KeyReader() as reader, screen:
            while True:
                tick = time.monotonic()
                term.feed(reader.read())
                console.tick()
                time.sleep(max(0.0, 0.05 - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        pass
    finally:
        if hardware is not None:
            hardware.close()


def cmd_panel(args: argparse.Namespace) -> None:
    from swarm.panel import Led, SwitchPanel, find_panel, led_report

    if find_panel() is None:
        raise SystemExit("no switch panel (06a3:0d67) in /sys/class/hidraw; attach it with usbipd")
    panel = SwitchPanel()
    print("Flip switches to see them. Gear LEDs: nose green, left red, right amber. Ctrl-C quits.")
    try:
        while True:
            state = panel.poll()
            if state is not None:
                print(f"{state.bits:06x}  {state.describe()}", flush=True)
            panel.set_leds(led_report(Led.GREEN, Led.RED, Led.AMBER))
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        panel.close()


def _view_server(
    args: argparse.Namespace,
    markers: list["Marker"],
    replay: Path | None = None,
    speed: float = 1.0,
    loop: bool = False,
) -> "ViewServer":
    from swarm.view import ViewConfig, ViewServer

    settings = _settings(args)
    fence = settings.fence
    config = ViewConfig(
        markers=markers,
        land_radius_m=settings.swarm.landing_zone_radius_m,
        leader_radius_m=settings.swarm.liftoff_max_leader_distance_m,
        point_b=settings.field.point_b,
        fence=list(fence.corners) if fence.enabled else [],
        vtx_mw=settings.sim.vtx_mw,
        host=args.view_host,
        http_port=args.view_port,
    )
    origin = settings.field.home
    return ViewServer(
        config, fallback_origin=origin, replay=replay, replay_speed=speed, replay_loop=loop
    )


def _settings(args: argparse.Namespace) -> "Settings":
    """mission.toml, or the --settings file, with this run's flags over it. Loaded once."""
    from dataclasses import replace

    from swarm.settings import SETTINGS_PATH, load_settings

    cached: Settings | None = getattr(args, "loaded_settings", None)
    if cached is not None:
        return cached
    settings = load_settings(Path(args.settings) if args.settings else SETTINGS_PATH)
    field = settings.field
    home: tuple[float, float, float | None] | None = getattr(args, "home", None)
    if home is not None:
        field = field.moved_to(*home)
    field = field.with_points(getattr(args, "point_a", None), getattr(args, "point_b", None))
    settings = replace(settings, field=field)
    if getattr(args, "count", None) is not None:
        settings = replace(settings, copters=args.count)
    if getattr(args, "follower_distance", None) is not None:
        settings = replace(
            settings, swarm=replace(settings.swarm, follower_distance_m=args.follower_distance)
        )
    if getattr(args, "router", None) is not None:
        settings = replace(settings, sim=replace(settings.sim, router=args.router))
    args.loaded_settings = settings
    return settings


def _launcher(
    link: MavlinkLink,
    leader: int,
    followers: list[int],
    settings: "Settings",
    own_mode_switch: bool,
) -> "Launcher":
    from swarm.launch import Launcher

    swarm = settings.swarm
    return Launcher(
        link,
        leader,
        followers,
        radius_m=swarm.follower_distance_m,
        alt_step_m=swarm.follower_height_step_m,
        dist_max_m=swarm.follow_max_distance_m,
        control_radius_m=swarm.liftoff_max_leader_distance_m,
        leader_min_alt_m=swarm.liftoff_min_leader_height_m,
        fence=settings.fence,
        own_mode_switch=own_mode_switch,
    )


def _field_markers(args: argparse.Namespace) -> list["Marker"]:
    """Point A for the 3D viewer. Point B reaches it in the watcher's telemetry."""
    from swarm.view import Marker

    lat, lon = _settings(args).field.point_a
    return [Marker(name="A", lat=lat, lon=lon)]


def _parse_home(text: str) -> tuple[float, float, float | None]:
    try:
        values = [float(x) for x in text.split(",")]
    except ValueError:
        values = []
    if len(values) not in (2, 3):
        raise argparse.ArgumentTypeError(
            f"expected LAT,LON or LAT,LON,ALT (ground elevation in metres AMSL), got {text!r}"
        ) from None
    return values[0], values[1], values[2] if len(values) == 3 else None


def _parse_point(text: str) -> tuple[float, float]:
    try:
        lat, lon = (float(x) for x in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected LAT,LON, got {text!r}") from None
    return lat, lon


def _parse_marker(text: str) -> "Marker":
    from swarm.view import Marker

    name, _, coords = text.partition("=")
    lat, _, lon = coords.partition(",")
    try:
        return Marker(name=name, lat=float(lat), lon=float(lon))
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected NAME=LAT,LON, got {text!r}") from None


def cmd_view(args: argparse.Namespace) -> None:
    markers: list[Marker] = _field_markers(args) + (args.mark or [])
    replay = Path(args.replay) if args.replay else None
    server = _view_server(args, markers, replay, args.speed, args.loop)
    try:
        server.start()
    except OSError as exc:
        raise SystemExit(f"3D viewer: {exc}; is another viewer or `swarm sim` running?") from None
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


def cmd_wingsim(args: argparse.Namespace) -> None:
    """The TBS Chupito on INAV SITL, flown by the drone radio or a demo pilot (swarm.wingrun),
    drawn on the 3D viewer's wing.html."""
    from swarm import wingrun
    from swarm.view import Marker
    from swarm.wingsim.airframe import REFERENCE
    from swarm.wingsim.weather import Weather

    settings = _settings(args)
    field, sim = settings.field, settings.sim
    lat, lon = args.at or field.point_a
    weather = Weather(
        speed=sim.wind_speed_ms if args.wind is None else args.wind,
        from_deg=sim.wind_from_deg if args.wind_from is None else args.wind_from,
        gusts=sim.wind_turbulence_ms if args.gusts is None else args.gusts,
        span=REFERENCE.span,
    )
    view = None if args.no_view else _view_server(args, [Marker(name="A", lat=lat, lon=lon)])
    wingrun.run((lat, lon, field.home_alt_m or 0.0), args.heading, args.demo, view, weather=weather)


def cmd_drive(args: argparse.Namespace) -> None:
    """The 3D viewer alone, for one of its demo pages (`args.page`): drive.html, a car on the
    map's 3D tiles, or wing.html, the TBS Chupito to fly."""
    server = _view_server(args, _field_markers(args))
    try:
        server.start()
    except OSError as exc:
        raise SystemExit(
            f"3D viewer: {exc}; if `swarm sim` or `swarm view` is running, open its {args.page}"
        ) from None
    at = f"?at={args.at[0]},{args.at[1]}" if args.at else ""
    logging.info("open %s%s%s", server.url, args.page, at)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


def cmd_sim(args: argparse.Namespace) -> None:
    """The sim stack, the watcher and the 3D viewer until Ctrl-C: fly it with `swarm pilot`."""
    with _watched_sim(args):
        logging.info("stack running; fly it with `swarm pilot`; Ctrl-C to stop")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass


@contextmanager
def _watched_sim(args: argparse.Namespace) -> "Iterator[ViewServer | None]":
    """The sim stack, a watcher feeding the 3D viewer, and the viewer. Once the stack is coming
    down, Ctrl-C is ignored, so a second one cannot leave SITL instances running."""
    from swarm.osd import OsdFeed
    from swarm.physics import PhysicsFeed
    from swarm.sim import SimStack

    settings = _settings(args)
    viewer = None if args.no_view else _view_server(args, _field_markers(args))
    if viewer is not None:
        try:
            viewer.start()
        except OSError as exc:
            logging.warning("3D viewer not started (%s); try --view-port", exc)
            viewer = None
    stack = SimStack(
        sim_field=settings.field,
        count=settings.copters,
        spawn_spacing_m=settings.sim.spawn_spacing_m,
        sitl_params=settings.sim.sitl_params(),
        start_qgc=args.qgc,
        wipe_params=not args.keep_params,
        router=settings.sim.router,
    )
    stop_watch = threading.Event()
    watcher: threading.Thread | None = None
    physics: PhysicsFeed | None = None
    osd: OsdFeed | None = None
    try:
        stack.start()
        if viewer is not None:
            copters = list(range(1, settings.copters + 1))
            physics = PhysicsFeed(copters, stack.home_alt_m)
            physics.start()
            viewer.physics = physics
            osd = OsdFeed(copters)
            osd.start()
            viewer.osd = osd
            viewer.sim = stack
        # The pilot's link reports vehicles, their messages and warnings; the watcher's would
        # repeat every one.
        logging.getLogger(WATCH_LINK_LOG).setLevel(logging.ERROR)
        watching = _build_watcher([WATCHER_ENDPOINT], args)
        watcher = threading.Thread(
            target=watching.run_until, args=(stop_watch.is_set, float("inf")), daemon=True
        )
        watcher.start()
        yield viewer
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        stop_watch.set()
        if watcher is not None:
            watcher.join(timeout=2)
        stack.stop()
        if viewer is not None:
            viewer.stop()
        if physics is not None:
            physics.stop()
        if osd is not None:
            osd.stop()


def cmd_fly(args: argparse.Namespace) -> None:
    """The sim stack, the watcher and the 3D viewer, and the radio and the switch panel flying
    it, in one terminal."""
    with _watched_sim(args):
        _run_pilot(args, [TOOLS_ENDPOINT])


def cmd_rehearse(args: argparse.Namespace) -> None:
    """The whole mission on the sim stack, with a scripted switch panel and radio."""
    from swarm.console import Console
    from swarm.launch import KEYS
    from swarm.rehearse import ScriptedPanel, ScriptedRadio, rehearse, summary

    settings = _settings(args)
    if settings.copters < 2:
        raise SystemExit("rehearse: 2 copters or more, the leader and at least one follower")
    ok = False
    with _watched_sim(args) as viewer:
        link = MavlinkLink([TOOLS_ENDPOINT])
        followers = [s for s in range(2, settings.copters + 1)][: len(KEYS)]
        launcher = _launcher(link, 1, followers, settings, own_mode_switch=True)
        panel, radio = ScriptedPanel(), ScriptedRadio([1, *followers])
        term, screen = _terminal_panel(panel, launcher, args.no_terminal_panel, interactive=False)
        radio.start()
        try:
            with screen or nullcontext():
                console = Console(launcher, term or panel, radio)
                results = rehearse(
                    console,
                    panel,
                    radio,
                    viewer,
                    settings.copters,
                    settings.field,
                    settings.swarm.landing_zone_radius_m,
                )
        except KeyboardInterrupt:
            results = []
        finally:
            radio.stop()
        print("\n".join(["", *summary(results)]), flush=True)
        ok = bool(results) and all(r.ok for r in results)
        if args.stay:
            logging.info("stack stays up for a look in the 3D viewer; Ctrl-C to stop")
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass
    if not ok:
        raise SystemExit(1)


def cmd_plot(args: argparse.Namespace) -> None:
    from swarm.plot import plot_log

    plot_log(Path(args.log), Path(args.out) if args.out else None)


def _add_count_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--count", type=int, help="copters, the leader included (mission.toml: copters)")
    p.add_argument(
        "--follower-distance",
        type=float,
        help="each follower's distance from the leader, metres (mission.toml: follower_distance_m)",
    )


def _add_pilot_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--leader", type=int, default=1, help="system id of the piloted drone")
    p.add_argument("--device", help="radio input device (default: first in /dev/input/by-id)")
    p.add_argument("--no-radio", action="store_true", help="keys only; fly the leader otherwise")
    p.add_argument(
        "--no-panel", action="store_true", help="ignore the switch panel, the terminal's too"
    )
    _add_terminal_panel_arg(p)


def _add_terminal_panel_arg(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--no-terminal-panel",
        action="store_true",
        help="do not draw the switch panel at the foot of the terminal",
    )


def _add_view_args(p: argparse.ArgumentParser) -> None:
    from swarm.view import DEFAULT_HTTP_PORT

    p.add_argument(
        "--view-port", type=int, default=DEFAULT_HTTP_PORT, help="HTTP port of the 3D viewer"
    )
    p.add_argument(
        "--view-host",
        default="127.0.0.1",
        help="address the 3D viewer listens on; 0.0.0.0 to open it from other devices",
    )


def _add_router_args(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--router",
        choices=["sitl", "field"],
        help="sitl: router/sitl.conf over SITL's TCP ports; field: router/field.conf, each "
        "copter sending MAVLink over UDP as the drones' Wi-Fi bridges do (mission.toml: "
        "[sim] router)",
    )


def _add_settings_arg(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--settings",
        metavar="PATH",
        help="the mission's settings file (default: mission.toml); flags override it",
    )


def _add_field_args(p: argparse.ArgumentParser) -> None:
    _add_settings_arg(p)
    p.add_argument(
        "--home",
        type=_parse_home,
        metavar="LAT,LON[,ALT]",
        help="the sim's home: copter 1 starts here (ALT is ground elevation AMSL, looked up "
        "online without it); A and B keep their offsets from it unless given too "
        "(mission.toml: [sim] home)",
    )
    p.add_argument(
        "--point-a",
        type=_parse_point,
        metavar="LAT,LON",
        help="point A, where the swarm flies in formation and holds (mission.toml: point_a)",
    )
    p.add_argument(
        "--point-b",
        type=_parse_point,
        metavar="LAT,LON",
        help="point B, where each drone is flown on to and landed (mission.toml: point_b)",
    )


def main() -> None:
    parser = argparse.ArgumentParser(prog="swarm")
    parser.add_argument("-v", "--verbose", action="store_true")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser(
        "watch", help="publish telemetry for the 3D viewer without commanding any vehicle"
    )
    p.add_argument(
        "--conn", action="append", help="MAVLink connection string (default: router UDP 14560)"
    )
    _add_field_args(p)
    p.set_defaults(func=cmd_watch)

    p = sub.add_parser(
        "pilot",
        help="fly the leader with a USB radio (no QGC); launch followers with keys a-d or "
        "run the mission from the switch panel",
    )
    p.add_argument(
        "--conn", action="append", help="MAVLink connection string (default: router UDP 14570)"
    )
    _add_settings_arg(p)
    _add_count_args(p)
    _add_pilot_args(p)
    p.set_defaults(func=cmd_pilot)

    p = sub.add_parser("panel", help="show the switch panel's switches and light its gear LEDs")
    p.set_defaults(func=cmd_panel)

    p = sub.add_parser(
        "panel-demo",
        help="the switch panel in the terminal flying pretend drones; no sim, no radio",
    )
    _add_settings_arg(p)
    p.add_argument("--no-panel", action="store_true", help="ignore the hardware switch panel")
    p.set_defaults(func=cmd_panel_demo)

    p = sub.add_parser(
        "sim",
        help="start the sim stack (SITL copters, mavlink-router), the watcher and the 3D viewer; "
        "fly it with `swarm pilot`",
    )
    p.add_argument("--count", type=int, help="copters to start, up to 5 (mission.toml: copters)")
    _add_field_args(p)
    p.add_argument(
        "--qgc", action="store_true", help="also start QGroundControl (backup display and control)"
    )
    p.add_argument(
        "--keep-params", action="store_true", help="keep each copter's stored parameters"
    )
    _add_router_args(p)
    p.add_argument("--no-view", action="store_true", help="do not start the 3D viewer")
    _add_view_args(p)
    p.set_defaults(func=cmd_sim)

    p = sub.add_parser(
        "fly",
        help="one command: the sim, the 3D viewer, and the radio and switch panel flying it",
    )
    _add_count_args(p)
    _add_field_args(p)
    _add_pilot_args(p)
    p.add_argument(
        "--qgc", action="store_true", help="also start QGroundControl (backup display and control)"
    )
    p.add_argument(
        "--keep-params", action="store_true", help="keep each copter's stored parameters"
    )
    _add_router_args(p)
    p.add_argument("--no-view", action="store_true", help="do not start the 3D viewer")
    _add_view_args(p)
    p.set_defaults(func=cmd_fly)

    p = sub.add_parser(
        "rehearse",
        help="fly the whole mission on the sim with a scripted switch panel and radio, "
        "checking every step; exit status 1 on the first that fails",
    )
    _add_count_args(p)
    _add_field_args(p)
    p.add_argument(
        "--qgc", action="store_true", help="also start QGroundControl (backup display and control)"
    )
    p.add_argument(
        "--keep-params", action="store_true", help="keep each copter's stored parameters"
    )
    _add_router_args(p)
    p.add_argument("--no-view", action="store_true", help="do not start the 3D viewer")
    p.add_argument(
        "--stay", action="store_true", help="keep the stack up after the run until Ctrl-C"
    )
    _add_terminal_panel_arg(p)
    _add_view_args(p)
    p.set_defaults(func=cmd_rehearse)

    p = sub.add_parser(
        "view", help="3D mission viewer for live telemetry (UDP 14600) or a replayed log"
    )
    p.add_argument("--replay", help="telemetry log (logs/*.jsonl) to play instead of live data")
    p.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    p.add_argument("--loop", action="store_true", help="restart the replay when it ends")
    p.add_argument(
        "--mark",
        action="append",
        type=_parse_marker,
        help="extra map marker NAME=LAT,LON (repeatable)",
    )
    _add_field_args(p)
    _add_view_args(p)
    p.set_defaults(func=cmd_view)

    p = sub.add_parser(
        "drive", help="drive a car on the map's Google 3D tiles, with physics (demo)"
    )
    p.add_argument(
        "--at", type=_parse_point, metavar="LAT,LON", help="start here instead of point A"
    )
    _add_field_args(p)
    _add_view_args(p)
    p.set_defaults(func=cmd_drive, page="drive.html")

    p = sub.add_parser("wing", help="fly the TBS Chupito, a fixed wing, on the map (demo)")
    p.add_argument(
        "--at", type=_parse_point, metavar="LAT,LON", help="start here instead of point A"
    )
    _add_field_args(p)
    _add_view_args(p)
    p.set_defaults(func=cmd_drive, page="wing.html")

    p = sub.add_parser(
        "wingsim", help="fly the TBS Chupito on INAV SITL with its own flight model (radio)"
    )
    p.add_argument(
        "--at", type=_parse_point, metavar="LAT,LON", help="throw it here instead of point A"
    )
    p.add_argument("--heading", type=float, default=0.0, help="facing this way, degrees true")
    p.add_argument("--demo", action="store_true", help="no radio: arm, throw and circle on its own")
    p.add_argument("--no-view", action="store_true", help="no 3D viewer: the terminal only")
    p.add_argument("--wind", type=float, metavar="M/S", help="wind at 6 m (default: mission.toml)")
    p.add_argument("--wind-from", type=float, metavar="DEG", help="where it blows from, true")
    p.add_argument("--gusts", type=float, metavar="M/S", help="gusts' peak above the wind")
    _add_field_args(p)
    _add_view_args(p)
    p.set_defaults(func=cmd_wingsim)

    p = sub.add_parser("plot", help="plot a telemetry log to a PNG")
    p.add_argument("log")
    p.add_argument("--out")
    p.set_defaults(func=cmd_plot)

    args = parser.parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    from swarm.settings import SettingsError
    from swarm.sim import SimStackError

    try:
        args.func(args)
    except SimStackError as exc:
        raise SystemExit(f"sim: {exc}") from None
    except SettingsError as exc:
        raise SystemExit(f"settings: {exc}") from None
