"""The page's frame time, played for real (DESIGN.md §3.2, §14): headless Chrome on the GPU at
the user's window size, uncapped, through Shoot Down (walk, sprint, a quick string of shots) and
the truck in Free Fly (drive). Each step's frames, the frame loop's dearest systems and the
longest frame's make-up (Chrome's long-animation-frame record: which scripts, how long the
render) are printed, and a frame over budget fails it.

    uvx --with playwright python scripts/perf.py [shoot] [truck] [--map free|google]

The map: `free` (the default), Esri's imagery and terrain, no key, served on port 8610 with
vehicle slot 6, so it runs beside the game; `google`, Google's 3D tiles, which answer only on
port 8600 and cost money. Run by `scripts/check.sh --perf` (both). It serves the built page
itself and stops it after. Shoot Down starts ArduCopter SITL: about a minute before the drone
is up.
"""

import json
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

from playwright.sync_api import Page, sync_playwright

ROOT = Path(__file__).resolve().parents[1]
MAPS = {"free": (8610, 6), "google": (8600, 0)}
URL = "http://localhost:8610/"
W, H = 3440, 1400
# Budgets, ms: the 95th percentile frame, the longest, and any one system's 95th percentile
# (Cesium's own render is the frame's floor and is held only by the frame budgets).
BUDGET = {"p95": 40.0, "max": 250.0, "system_p95": 6.0}
NOT_SYSTEMS = {"cesium render"}

LOG = """() => { window.__frames = []; window.__loaf = []; let last = performance.now();
  const tick = (t) => { window.__frames.push(t - last); last = t; requestAnimationFrame(tick); };
  requestAnimationFrame(tick); }"""

# Chrome's record of each long animation frame (over 50 ms): when its scripts ran and what
# called them, and when it rendered. From page load, so the first click's is caught.
LOAF = """
window.__loaf = [];
new PerformanceObserver((list) => {
  for (const e of list.getEntries()) {
    window.__loaf.push({
      at: Math.round(e.startTime),
      ms: Math.round(e.duration),
      block: Math.round(e.blockingDuration),
      render: Math.round(e.renderStart ? e.startTime + e.duration - e.renderStart : 0),
      style: Math.round(
        e.styleAndLayoutStart ? e.startTime + e.duration - e.styleAndLayoutStart : 0),
      scripts: e.scripts.map((x) => ({
        ms: Math.round(x.duration),
        what: `${x.invokerType} ${x.invoker} ${x.sourceFunctionName || ""}`.trim(),
        where: `${(x.sourceURL || "").split("/").pop()}:${x.sourceCharPosition}`,
      })).sort((a, b) => b.ms - a.ms).slice(0, 4),
    });
  }
}).observe({ type: "long-animation-frame", buffered: true });
"""


def step(page: Page, label: str, fails: list[str]) -> None:
    """The frames since the last step, and the loop's systems over the last two seconds."""
    take = "() => { const f = window.__frames; window.__frames = []; return f; }"
    dts = sorted(page.evaluate(take))
    if not dts:
        print(f"{label:22s} no frames", flush=True)
        fails.append(f"{label}: no frames")
        return
    p50, p95, most = dts[len(dts) // 2], dts[int(len(dts) * 0.95)], dts[-1]
    stats = page.evaluate("() => window.duels.perf.stats()")
    systems = [s for s in stats["systems"] if s["p95"] >= 0.1][:6]
    worst = max(stats["systems"], key=lambda s: s["max"], default=None)
    print(
        f"{label:22s} {len(dts):4d} frames  {1000 / p50:5.0f} fps  p50 {p50:5.1f}  p95 {p95:5.1f}"
        f"  max {most:6.1f} ms | "
        + ", ".join(f"{s['name']} {s['p50']:.1f}/{s['p95']:.1f}" for s in systems)
        + (f" | longest: {worst['name']} {worst['max']:.0f}" if worst else ""),
        flush=True,
    )
    loaf = page.evaluate("() => { const l = window.__loaf; window.__loaf = []; return l; }")
    if loaf:
        f = max(loaf, key=lambda x: x["ms"])
        scripts = "; ".join(f"{x['what']} {x['where']} {x['ms']}" for x in f["scripts"])
        print(
            f"  longest frame {f['ms']} ms: render {f['render']} ms (style/layout {f['style']}),"
            f" scripts: {scripts or 'none'}",
            flush=True,
        )
    if p95 > BUDGET["p95"]:
        fails.append(f"{label}: frame p95 {p95:.1f} ms > {BUDGET['p95']}")
    if most > BUDGET["max"]:
        fails.append(f"{label}: longest frame {most:.0f} ms > {BUDGET['max']}")
    for s in stats["systems"]:
        if s["name"] not in NOT_SYSTEMS and s["p95"] > BUDGET["system_p95"]:
            fails.append(f"{label}: {s['name']} p95 {s['p95']:.1f} ms > {BUDGET['system_p95']}")
        if s["dropped"] > 0:
            fails.append(f"{label}: {s['name']} dropped {s['dropped'] * 1000:.0f} ms")


def menu(page: Page, mode: str, place: str, vehicle: str) -> None:
    page.goto(URL)
    page.wait_for_selector(".mode-btn", timeout=60_000)
    page.click(f".mode-btn:has-text('{mode}')")
    page.click(f".card:has-text('{place}')")
    page.wait_for_selector(f".card:has-text('{vehicle}')", timeout=10_000)
    page.click(f".card:has-text('{vehicle}')")


def hold(page: Page, keys: list[str], seconds: float) -> None:
    for k in keys:
        page.keyboard.down(k)
    time.sleep(seconds)
    for k in reversed(keys):
        page.keyboard.up(k)


SHOOTER = "window.duels.flight.mode.shooter"


def shooter(page: Page) -> dict[str, float]:
    return page.evaluate(
        f"() => {{ const s = {SHOOTER}; return {{ e: s.e, n: s.n, feet: s.feet, pitch: s.pitch,"
        " hits: window.duels.flight.mode.rounds.groundHits }; }"
    )


def shoot(page: Page, fails: list[str]) -> None:
    menu(page, "Shoot Down", "AreaXO", "FPV quad")
    for _ in range(240):
        text = page.evaluate("() => document.querySelector('.shoot-hud')?.innerText ?? ''")
        if "hovering" in text:
            break
        time.sleep(0.5)
    else:
        fails.append("shoot: the drone never hovered")
        return
    time.sleep(3)
    page.evaluate(LOG)
    page.mouse.click(W // 2, H // 2)
    time.sleep(2)
    step(page, "shoot: standing", fails)
    was = shooter(page)
    hold(page, ["KeyW"], 3)
    hold(page, ["KeyA"], 1.5)
    step(page, "shoot: walking", fails)
    now = shooter(page)
    walked = ((now["e"] - was["e"]) ** 2 + (now["n"] - was["n"]) ** 2) ** 0.5
    print(f"  walked {walked:.1f} m, feet {was['feet']:.2f} -> {now['feet']:.2f} m", flush=True)
    if walked < 2:
        fails.append(f"shoot: walked only {walked:.1f} m in 4.5 s")
    hold(page, ["ShiftLeft", "KeyW"], 3)
    step(page, "shoot: sprinting", fails)
    page.mouse.down(button="right")
    time.sleep(0.5)
    for _ in range(10):
        page.mouse.down()
        time.sleep(0.03)
        page.mouse.up()
        time.sleep(0.15)
    step(page, "shoot: 10 quick shots", fails)
    time.sleep(3)
    page.mouse.up(button="right")
    step(page, "shoot: after", fails)
    # Down at the ground a few metres ahead: the rounds must stop in it.
    # (The mouse is locked: each move is read as how far it went, so it goes on down.)
    page.mouse.move(W // 2, 0)
    for i in range(1, 60):
        if shooter(page)["pitch"] < -0.35:
            break
        page.mouse.move(W // 2, 20 * i)
    hits = shooter(page)["hits"]
    for _ in range(3):
        page.mouse.down()
        time.sleep(0.03)
        page.mouse.up()
        time.sleep(0.3)
    time.sleep(1)
    into = shooter(page)["hits"] - hits
    print(
        f"  3 shots at the ground (pitch {shooter(page)['pitch']:.2f}): {into} went in", flush=True
    )
    if into < 3:
        fails.append(f"shoot: {into} of 3 shots at the ground met it")


def truck(page: Page, fails: list[str]) -> None:
    menu(page, "Free Fly", "AreaXO", "Military truck")
    for _ in range(120):
        if page.evaluate("() => !!document.querySelector('.hud')") and not page.evaluate(
            "() => document.body.innerText.includes('Loading the ground')"
        ):
            break
        time.sleep(0.5)
    time.sleep(4)
    page.evaluate(LOG)
    time.sleep(2)
    step(page, "truck: standing", fails)
    page.keyboard.down("KeyW")
    time.sleep(4)
    speed = page.evaluate("() => window.duels.flight.body().speed")
    page.keyboard.up("KeyW")
    print(f"  truck at {speed:.1f} m/s", flush=True)
    if speed < 2:
        fails.append(f"truck: only {speed:.1f} m/s after 4 s of throttle")
    step(page, "truck: driving", fails)
    hold(page, ["KeyW", "KeyD"], 3)
    step(page, "truck: turning", fails)


SCENARIOS = {"shoot": shoot, "truck": truck}


def served() -> bool:
    try:
        urllib.request.urlopen(URL, timeout=1)
        return True
    except OSError:
        return False


def main() -> int:
    global URL
    args = sys.argv[1:]
    which = args[args.index("--map") + 1] if "--map" in args else "free"
    if which not in MAPS:
        print(f"--map: one of {', '.join(MAPS)}", file=sys.stderr)
        return 2
    port, slot = MAPS[which]
    URL = f"http://localhost:{port}/"
    picked = [a for a in args if a in SCENARIOS] or list(SCENARIOS)
    if served():
        print(f"port {port} is taken: stop what serves it first", file=sys.stderr)
        return 2
    print(f"map: {which}, port {port}", flush=True)
    env = {**os.environ, "DRONE_DUELS_PREFS": str(ROOT / "logs" / "perf-prefs.json")}
    if which == "free":
        env.update(GOOGLE_MAPS_API_KEY="", CESIUM_ION_TOKEN="")
    log = open(ROOT / "logs" / f"perf-server-{which}.log", "w")  # noqa: SIM115
    server = subprocess.Popen(
        ["uv", "run", "duels", "serve", "--no-build", "--port", str(port), "--slot", str(slot)],
        cwd=ROOT,
        stdout=log,
        stderr=subprocess.STDOUT,
        env=env,
    )
    fails: list[str] = []
    try:
        for _ in range(60):
            if served():
                break
            time.sleep(0.5)
        with sync_playwright() as pw:
            browser = pw.chromium.launch(
                channel="chrome",
                headless=True,
                args=[
                    "--enable-gpu",
                    "--ignore-gpu-blocklist",
                    "--use-angle=gl",
                    "--disable-frame-rate-limit",
                    "--disable-gpu-vsync",
                    "--autoplay-policy=no-user-gesture-required",
                ],
            )
            for name in picked:
                page = browser.new_page(viewport={"width": W, "height": H})
                page.add_init_script(LOAF)
                errors: list[str] = []
                page.on("pageerror", lambda e, errors=errors: errors.append(str(e)))
                SCENARIOS[name](page, fails)
                fails.extend(f"{name}: page error {e}" for e in errors[:3])
                page.close()
            browser.close()
    finally:
        server.terminate()
        try:
            server.wait(timeout=30)
        except subprocess.TimeoutExpired:
            server.kill()
        log.close()
    print(json.dumps({"fails": fails}, indent=1) if fails else "perf: within budget")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
