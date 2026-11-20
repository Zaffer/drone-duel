"""`duels`: the game's command line.

    duels                 build the page if it changed, serve it and open Chrome
    duels serve           the same, without opening Chrome (--open to open it)
    duels doctor          check the setup and every content file, and say what is missing
    duels gen-protocol    write the protocol's JSON Schema for the page's types
    duels gen-web         write what is checked in for the web version and generated: its
                          tests' fixtures, and the plant's C (drone_duels.webdata.generated)
    duels web             build the web version (web/dist-web/: a static page, its simulator in
                          a worker) and serve it; --build-only to only build it
"""

import argparse
import json
import logging
import shutil
import subprocess
import sys
import webbrowser
from pathlib import Path

from drone_duels import ports

ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "web"
SCHEMA = WEB / "src" / "net" / "protocol.schema.json"


def bun() -> str | None:
    found = shutil.which("bun")
    if found:
        return found
    home = Path.home() / ".bun" / "bin" / "bun"
    return str(home) if home.exists() else None


def page_stale() -> bool:
    built = WEB / "dist" / "index.html"
    if not built.exists():
        return True
    stamp = built.stat().st_mtime
    sources = [*(WEB / "src").rglob("*"), WEB / "index.html", WEB / "package.json"]
    return any(p.is_file() and p.stat().st_mtime > stamp for p in sources)


def build_page(force: bool = False) -> None:
    if not force and not page_stale():
        return
    b = bun()
    if b is None:
        raise SystemExit("no bun: run scripts/setup.sh")
    print("building the page...", flush=True)
    subprocess.run([b, "run", "build"], cwd=WEB, check=True)


def serve(port: int, open_browser: bool, build: bool, slot: int = 0) -> None:
    import uvicorn

    from drone_duels.server.app import create_app

    if build:
        build_page()
    url = f"http://localhost:{port}/"
    if open_browser:
        chrome = shutil.which("google-chrome") or shutil.which("chromium")
        if chrome:
            subprocess.Popen([chrome, f"--app={url}"], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
        else:
            webbrowser.open(url)
    print(f"Drone Duels at {url}", flush=True)
    uvicorn.run(create_app(slot=slot), host="127.0.0.1", port=port, log_level="warning")


def gen_web(check: bool) -> None:
    from drone_duels import webdata

    stale: list[Path] = []
    for path, text in webdata.generated().items():
        if path.exists() and same_generated(path.read_text(), text):
            continue
        stale.append(path)
        if not check:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
            print(f"wrote {path.relative_to(ROOT)}")
    if check and stale:
        names = ", ".join(str(p.relative_to(ROOT)) for p in stale)
        raise SystemExit(f"{names}: stale: run `uv run duels gen-web`")


def same_generated(have: str, want: str, tolerance: float = 1e-9) -> bool:
    """Whether a generated file is current: the same text, or (JSON) the same but for numbers
    within `tolerance`, relative: the reference flight's floats differ in their last digits
    from one machine's maths library to the next."""
    if have == want:
        return True
    try:
        a, b = json.loads(have), json.loads(want)
    except ValueError:
        return False

    def close(x: object, y: object) -> bool:
        if isinstance(x, bool) or isinstance(y, bool):
            return x is y
        if isinstance(x, int | float) and isinstance(y, int | float):
            return abs(x - y) <= tolerance * max(1.0, abs(x), abs(y))
        if isinstance(x, list) and isinstance(y, list):
            xs: list[object] = x  # pyright: ignore[reportUnknownVariableType]
            ys: list[object] = y  # pyright: ignore[reportUnknownVariableType]
            return len(xs) == len(ys) and all(close(p, q) for p, q in zip(xs, ys, strict=True))
        if isinstance(x, dict) and isinstance(y, dict):
            xd: dict[object, object] = x  # pyright: ignore[reportUnknownVariableType]
            yd: dict[object, object] = y  # pyright: ignore[reportUnknownVariableType]
            return xd.keys() == yd.keys() and all(close(xd[k], yd[k]) for k in xd)
        return x == y

    return close(a, b)


WEB_DIST = WEB / "dist-web"
SIM_MODULE = ROOT / "external" / "web" / "sim" / "sim.wasm"


def build_web() -> None:
    """The web version, whole, in web/dist-web/: the page built for it, and its data."""
    from drone_duels import webdata

    b = bun()
    if b is None:
        raise SystemExit("no bun: run scripts/setup.sh")
    if not SIM_MODULE.exists():
        raise SystemExit(f"no simulator at {SIM_MODULE}: run scripts/setup-web.sh")
    print("building the web version...", flush=True)
    subprocess.run([b, "run", "build", "--web"], cwd=WEB, check=True)
    webdata.write(WEB_DIST / "data")
    print(f"wrote {WEB_DIST.relative_to(ROOT)}")


def serve_web(port: int, open_browser: bool) -> None:
    """Serve web/dist-web/ as a static host would (GitHub Pages), from a folder, not the root,
    so a path that only works from the root shows here first."""
    import functools
    import http.server

    class Handler(http.server.SimpleHTTPRequestHandler):
        extensions_map = {
            **http.server.SimpleHTTPRequestHandler.extensions_map,
            ".wasm": "application/wasm",
            ".js": "text/javascript",
        }

        def translate_path(self, path: str) -> str:
            if not path.startswith(WEB_PREFIX):
                return str(WEB_DIST / "404")
            return super().translate_path(path[len(WEB_PREFIX) - 1 :])

        def do_GET(self) -> None:
            if self.path in ("/", WEB_PREFIX.rstrip("/")):
                self.send_response(302)
                self.send_header("Location", WEB_PREFIX)
                self.end_headers()
                return
            super().do_GET()

        def log_message(self, format: str, *args: object) -> None:
            pass

    url = f"http://localhost:{port}{WEB_PREFIX}"
    handler = functools.partial(Handler, directory=str(WEB_DIST))
    with http.server.ThreadingHTTPServer(("127.0.0.1", port), handler) as httpd:
        if open_browser:
            webbrowser.open(url)
        print(f"Drone Duels (web) at {url}", flush=True)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


WEB_PREFIX = "/drone-duels/"
"""The folder the web version is served from here, as GitHub Pages serves a repository's."""


def gen_protocol(check: bool) -> None:
    from drone_duels.server.protocol import schema

    text = json.dumps(schema(), indent=2) + "\n"
    if check:
        if not SCHEMA.exists() or SCHEMA.read_text() != text:
            raise SystemExit(f"{SCHEMA} is stale: run `uv run duels gen-protocol`")
        return
    SCHEMA.parent.mkdir(parents=True, exist_ok=True)
    SCHEMA.write_text(text)
    b = bun()
    if b:
        subprocess.run([b, "run", "gen:protocol"], cwd=WEB, check=True)
    print(f"wrote {SCHEMA.relative_to(ROOT)}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="duels", description="Drone Duels")
    parser.add_argument("-v", "--verbose", action="store_true")
    sub = parser.add_subparsers(dest="command")
    p = sub.add_parser("serve", help="serve the game (without opening Chrome)")
    p.add_argument("--port", type=int, default=ports.HTTP)
    p.add_argument("--open", action="store_true", help="open Chrome on it")
    p.add_argument("--no-build", action="store_true", help="serve web/dist as it is")
    p.add_argument(
        "--slot", type=int, default=0,
        help="the vehicle slot (ports); a second game on this machine needs another, e.g. 1",
    )
    sub.add_parser("doctor", help="check the setup and the content files")
    p = sub.add_parser("gen-protocol", help="write the protocol's JSON Schema (and TS types)")
    p.add_argument("--check", action="store_true", help="fail if it is stale instead")
    p = sub.add_parser("gen-web", help="write the web version's generated, checked-in files")
    p.add_argument("--check", action="store_true", help="fail if any is stale instead")
    p = sub.add_parser("web", help="build the web version (web/dist-web/) and serve it")
    p.add_argument("--port", type=int, default=ports.HTTP)
    p.add_argument("--build-only", action="store_true", help="build it and stop")
    p.add_argument("--no-build", action="store_true", help="serve web/dist-web as it is")
    p.add_argument("--open", action="store_true", help="open a browser on it")
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    if args.command is None:
        serve(ports.HTTP, open_browser=True, build=True)
    elif args.command == "serve":
        serve(args.port, open_browser=args.open, build=not args.no_build, slot=args.slot)
    elif args.command == "doctor":
        from drone_duels.doctor import doctor

        sys.exit(doctor())
    elif args.command == "gen-protocol":
        gen_protocol(args.check)
    elif args.command == "gen-web":
        gen_web(args.check)
    elif args.command == "web":
        if not args.no_build:
            build_web()
        if not args.build_only:
            serve_web(args.port, args.open)
