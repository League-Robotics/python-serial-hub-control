"""Hardware-free unit tests for the _fit_limit helper in examples/velocity_chart.py.

These tests cover the pure axis-scaling math without launching a matplotlib
GUI or connecting to a hub.

Coverage:
- Empty data uses floor as the minimum span.
- Setpoint larger than data drives the limit.
- Data larger than setpoint drives the limit.
- Margin (15%) is applied correctly.
- vmax_cap clamps the result from above when supplied.
- vmax_cap has no effect when the auto-scaled value is below the cap.
- floor prevents collapse to zero when motor is stopped.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
import types

import pytest


# ---------------------------------------------------------------------------
# Import _fit_limit without importing the rest of velocity_chart.py.
# The module imports matplotlib and rhsp at top level — we load just the
# helper via AST-safe source inspection + exec of the function body to
# avoid triggering the display-backend import.
# ---------------------------------------------------------------------------


def _load_fit_limit():
    """Return the _fit_limit function extracted from velocity_chart.py."""
    src_path = pathlib.Path(__file__).parent.parent / "examples" / "velocity_chart.py"
    src = src_path.read_text()

    # Build a minimal module namespace and exec only the _fit_limit function.
    # We locate the function definition by compiling the full source into an AST,
    # then re-exec only that function in an isolated namespace.
    import ast

    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_fit_limit":
            # Re-compile just this function definition.
            func_src = ast.get_source_segment(src, node)
            ns: dict = {"__builtins__": __builtins__}
            exec(compile(func_src, src_path.name, "exec"), ns)  # noqa: S102
            return ns["_fit_limit"]
    raise RuntimeError("_fit_limit not found in velocity_chart.py")


_fit_limit = _load_fit_limit()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_empty_data_returns_floor_plus_margin():
    """With no data and setpoint=0, result equals floor * 1.15."""
    result = _fit_limit([], setpoint=0.0, floor=50.0)
    assert result == pytest.approx(50.0 * 1.15)


def test_setpoint_larger_than_data():
    """When setpoint dominates, result is setpoint * 1.15."""
    result = _fit_limit([10, -5, 20], setpoint=500.0, floor=50.0)
    assert result == pytest.approx(500.0 * 1.15)


def test_data_larger_than_setpoint():
    """When a data value dominates, result is max(|data|) * 1.15."""
    result = _fit_limit([800, -900, 100], setpoint=200.0, floor=50.0)
    assert result == pytest.approx(900.0 * 1.15)


def test_floor_prevents_collapse():
    """When all data and setpoint are near zero, floor applies."""
    result = _fit_limit([1, -1], setpoint=1.0, floor=50.0)
    assert result == pytest.approx(50.0 * 1.15)


def test_custom_margin():
    """Custom margin fraction is applied correctly."""
    result = _fit_limit([100], setpoint=0.0, margin=0.20, floor=50.0)
    assert result == pytest.approx(100.0 * 1.20)


def test_vmax_cap_clamps_from_above():
    """Explicit vmax_cap acts as a ceiling."""
    # Without cap: 1000 * 1.15 = 1150; cap at 800 → 800.
    result = _fit_limit([1000], setpoint=0.0, floor=50.0, vmax_cap=800.0)
    assert result == pytest.approx(800.0)


def test_vmax_cap_has_no_effect_when_below():
    """vmax_cap does not truncate when auto-scale is below it."""
    # Auto-scale: 100 * 1.15 = 115; cap at 2000 → 115.
    result = _fit_limit([100], setpoint=0.0, floor=50.0, vmax_cap=2000.0)
    assert result == pytest.approx(100.0 * 1.15)


def test_negative_setpoint_uses_absolute_value():
    """Negative setpoint treated as |setpoint| for span computation."""
    result = _fit_limit([], setpoint=-400.0, floor=50.0)
    assert result == pytest.approx(400.0 * 1.15)


def test_ratio_5_scenario():
    """Realistic scenario: --speed 1000 --ratio 5.0, motor B setpoint=5000."""
    # Motor B should scale to about 5000 * 1.15 = 5750.
    result_b = _fit_limit([], setpoint=5000.0, floor=50.0)
    assert result_b == pytest.approx(5750.0)

    # Motor A should scale to about 1000 * 1.15 = 1150.
    result_a = _fit_limit([], setpoint=1000.0, floor=50.0)
    assert result_a == pytest.approx(1150.0)

    # The two results must differ substantially.
    assert result_b > result_a * 4
