"""Tests for the public rhsp package API surface (ticket 001-013).

Verifies that all names promised in ``src/rhsp/__init__.py`` are importable
directly from the top-level ``rhsp`` package and are the correct types.
Hardware-free.
"""

from __future__ import annotations

import inspect


def test_connect_importable() -> None:
    from rhsp import connect
    assert callable(connect)


def test_enumerate_hubs_importable() -> None:
    from rhsp import enumerate_hubs
    assert callable(enumerate_hubs)


def test_hub_importable() -> None:
    from rhsp import Hub
    assert inspect.isclass(Hub)


def test_session_importable() -> None:
    from rhsp import Session
    assert inspect.isclass(Session)


def test_errors_importable() -> None:
    from rhsp import RhspError, ProtocolError, ChecksumError, RhspTimeoutError, NackError
    for cls in (RhspError, ProtocolError, ChecksumError, RhspTimeoutError, NackError):
        assert issubclass(cls, Exception), f"{cls.__name__} should be an Exception subclass"


def test_enums_importable() -> None:
    from rhsp import MotorMode, ZeroPowerBehavior, NackCode, ModuleStatusBits, MotorStatusBits
    import enum
    for e in (MotorMode, ZeroPowerBehavior, NackCode, ModuleStatusBits, MotorStatusBits):
        assert issubclass(e, enum.Enum), f"{e.__name__} should be an Enum"


def test_bulk_dataclasses_importable() -> None:
    from rhsp import BulkInputData, ModuleStatus
    import dataclasses
    assert dataclasses.is_dataclass(BulkInputData)
    assert dataclasses.is_dataclass(ModuleStatus)


def test_all_names_in_dunder_all() -> None:
    """Every name listed in __all__ must be importable from rhsp."""
    import rhsp
    for name in rhsp.__all__:
        assert hasattr(rhsp, name), f"rhsp.__all__ lists {name!r} but it is not present"


def test_key_import_combo() -> None:
    """The canonical quick-start import pattern must resolve without error."""
    from rhsp import connect, Hub, MotorMode, NackError  # noqa: F401
    from rhsp import Session, BulkInputData, ModuleStatus  # noqa: F401


def test_no_gui_entry_point() -> None:
    """src/rhsp has no __main__.py — python -m rhsp should not import tkinter."""
    import importlib.util
    spec = importlib.util.find_spec("rhsp.__main__")
    assert spec is None, (
        "rhsp.__main__ exists but should not — the GUI entry point was removed"
    )
