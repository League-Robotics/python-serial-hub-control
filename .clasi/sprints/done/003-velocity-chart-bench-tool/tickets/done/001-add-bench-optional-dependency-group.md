---
id: 003-001
title: Add bench optional dependency group to pyproject.toml
status: done
use-cases:
- SUC-001
depends-on: []
issue: plan-velocity-chart-py-bench-tool-for-serialhubcontrol-rhsp.md
---

# 003-001: Add bench optional dependency group to pyproject.toml

## Description

Add `matplotlib` and `numpy` as an optional dependency group (`bench`) in
`pyproject.toml` so the interactive bench tool can be run with
`uv run --extra bench python examples/velocity_chart.py` without adding
large plotting libraries to the core install or the dev (pytest) environment.

This ticket changes only `pyproject.toml`. No source code changes.

## Acceptance Criteria

- [x] `[project.optional-dependencies]` table exists in `pyproject.toml` with
      a `bench` key listing `matplotlib>=3.8` and `numpy>=1.26`.
- [x] `uv run --extra bench python -c "import matplotlib, numpy"` exits 0.
- [x] `uv run pytest` exits 0 with the same test count as before (no new
      failures, no new test collections).
- [x] `uv run python -c "import rhsp"` exits 0 (core install unaffected; no
      matplotlib/numpy in core deps).

## Implementation Plan

### Approach

Add one new table to `pyproject.toml`. The `[dependency-groups] dev` entry
(pytest) is left unchanged. The new entry is `[project.optional-dependencies]`
(PEP 508 standard extras, compatible with both uv and pip).

### Files to Modify

- `pyproject.toml` — add after the existing `[dependency-groups]` block:

```toml
[project.optional-dependencies]
bench = ["matplotlib>=3.8", "numpy>=1.26"]
```

### Testing Plan

1. Run `uv run --extra bench python -c "import matplotlib, numpy; print('ok')"`.
   Confirm exit 0 and "ok" printed.
2. Run `uv run pytest`. Confirm same pass count as before this ticket.
3. Run `uv run python -c "import rhsp; print('ok')"` (without `--extra bench`).
   Confirm matplotlib is not pulled in by the core install.

### Documentation Updates

None required. The `pyproject.toml` change is self-documenting via the table
name `bench`.
