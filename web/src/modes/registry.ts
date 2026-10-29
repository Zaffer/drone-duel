// The game's modes, in the main menu's order (DESIGN.md §7). A new mode is a ModeDef here.

import { duel } from "./duel/duel.ts";
import { freefly } from "./freefly.ts";
import { race } from "./race/race.ts";
import type { ModeDef } from "./types.ts";

export const MODES: ModeDef[] = [freefly, duel, race];
