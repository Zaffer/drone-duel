#!/usr/bin/env bash
# Every check, Python and the page (DESIGN.md §14). `--fast` skips the tests that start real
# firmware (pytest -m "not sitl"); `--perf` also plays the page headless on the GPU and holds its
# frame time to budget (scripts/perf.py: it serves on 8600, so stop the game first).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUN="$(command -v bun || echo "$HOME/.bun/bin/bun")"
cd "$ROOT"
uv run duels gen-protocol --check
uv run ruff check src tests tools
uv run basedpyright src tests tools
if [ "${1:-}" = "--fast" ]; then uv run pytest -q -m "not sitl"; else uv run pytest -q; fi
cd web
"$BUN" x tsc --noEmit
"$BUN" x biome check src scripts test
"$BUN" test
"$BUN" run build
if [ "${1:-}" = "--perf" ]; then
  cd "$ROOT"
  uvx --with playwright python scripts/perf.py
fi
echo "all checks passed"
