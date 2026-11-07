"""The page's frame time, played for real (DESIGN.md §3.2, §14): headless Chrome on the GPU at
the user's window size, uncapped, through Shoot Down (walk, sprint, a quick string of shots) and
the truck in Free Fly (drive). Each step's frames and the frame loop's dearest systems are
printed, and a frame over budget fails it.

    uvx --with playwright python scripts/perf.py [shoot] [truck]

Run by `scripts/check.sh --perf`. It serves the built page on port 8600 itself (Google's tiles
answer only there; they cost money, so this is not in the ordinary checks) and stops it after.
Shoot Down starts ArduCopter SITL: about a minute before the drone is up.
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
URL = "http://localhost:8600/"
W, H = 3440, 1400
# Budgets, ms: the 95th percentile frame, the longest, and any one system's 95th percentile
# (Cesium's own render is the frame's floor and is held only by the frame budgets).
BUDGET = {"p95": 40.0, "max": 250.0, "system_p95": 6.0}
NOT_SYSTEMS = {"cesium render"}

LOG = """() => { window.__frames = []; let last = performance.now();
  const tick = (t) => { window.__frames.push(t - last); last = t; requestAnimationFrame(tick); };
  requestAnimationFrame(tick); }"""


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
    print(
        f"{label:22s} {len(dts):4d} frames  {1000 / p50:5.0f} fps  p50 {p50:5.1f}  p95 {p95:5.1f}"
        f"  max {most:6.1f} ms | "
        + ", ".join(f"{s['name']} {s['p50']:.1f}/{s['p95']:.1f}" for s in systems),
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
    hold(page, ["KeyW"], 3)
    hold(page, ["KeyA"], 1.5)
    step(page, "shoot: walking", fails)
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
    hold(page, ["KeyW"], 4)
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
    picked = [a for a in sys.argv[1:] if a in SCENARIOS] or list(SCENARIOS)
    if served():
        print("port 8600 is taken: stop the game first", file=sys.stderr)
        return 2
    log = open(ROOT / "logs" / "perf-server.log", "w")  # noqa: SIM115
    server = subprocess.Popen(
        ["uv", "run", "duels", "serve", "--no-build"],
        cwd=ROOT,
        stdout=log,
        stderr=subprocess.STDOUT,
        env={**os.environ, "DRONE_DUELS_PREFS": str(ROOT / "logs" / "perf-prefs.json")},
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
