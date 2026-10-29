"""`duels`: the game's command line.

    duels                 build the page if it changed, serve it and open Chrome
    duels serve           the same, without opening Chrome (--open to open it)
    duels doctor          check the setup and every content file, and say what is missing
    duels gen-protocol    write the protocol's JSON Schema for the page's types
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
