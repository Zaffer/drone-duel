import { describe, expect, test } from "bun:test";
import { type Binding, codeLabel, hotkeys } from "../src/input/actions.ts";
import { shooterBindings } from "../src/shoot/controls.ts";
import { flightBindings, footBindings, menuBindings, overlayBindings } from "../src/ui/keys.ts";

/** Codes two tables mounted together both answer, with the same shift. */
function clashes(a: Record<string, Binding>, b: Record<string, Binding>): string[] {
  const out: string[] = [];
  for (const [na, x] of Object.entries(a)) {
    for (const [nb, y] of Object.entries(b)) {
      const sameShift = x.shift === undefined || y.shift === undefined || x.shift === y.shift;
      for (const c of x.codes) if (y.codes.includes(c) && sameShift) out.push(`${c}: ${na}/${nb}`);
    }
  }
  return out;
}

describe("the controls' tables", () => {
  test("codes read as the keys and buttons are marked", () => {
    expect(codeLabel("KeyR")).toBe("R");
    expect(codeLabel("Digit3")).toBe("3");
    expect(codeLabel("Escape")).toBe("Esc");
    expect(codeLabel("Mouse2")).toBe("Right button");
    expect(codeLabel("Pad7")).toBe("RT");
    expect(codeLabel("Pad10")).toBe("L3");
  });

  test("the Inputs tab shows a table's rows, keys then pad buttons, once each", () => {
    const rows = hotkeys(shooterBindings);
    expect(rows.find((r) => r.strip === "fire")?.keys).toEqual(["Left button", "RT"]);
    expect(rows.find((r) => r.strip === "sprint")?.keys).toEqual(["Shift", "L3"]);
    expect(rows.some((r) => r.does === "Walk forward")).toBe(false);
    const flight = hotkeys(flightBindings(["FPV", "Chase"]));
    expect(flight.find((r) => r.does.startsWith("Jump to a view"))?.keys).toEqual(["1–2"]);
    expect(flight.find((r) => r.does === "Previous view")?.keys).toEqual(["Shift+V"]);
  });

  test("tables mounted together never answer the same press twice", () => {
    expect(clashes(shooterBindings, footBindings())).toEqual([]);
    expect(
      clashes(
        flightBindings(["a", "b", "c", "d", "e", "f"]),
        overlayBindings(["1", "2", "3", "4", "5"]),
      ),
    ).toEqual([]);
    expect(clashes(menuBindings(["a", "b", "c", "d"]), {})).toEqual([]);
  });

  test("within the shooter's table, a key is one press's (Shift is sprint or breath: held)", () => {
    const own = Object.entries(shooterBindings);
    const seen = new Map<string, string>();
    for (const [name, b] of own) {
      for (const c of b.codes) {
        const other = seen.get(c);
        if (other && !(c.startsWith("Shift") && [name, other].sort().join() === "breath,sprint")) {
          throw new Error(`${c}: ${other} and ${name}`);
        }
        seen.set(c, name);
      }
    }
  });
});
