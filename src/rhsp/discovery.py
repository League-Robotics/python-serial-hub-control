"""Discovery — hub enumeration, connection, and QueryInterface.

Public API
----------
- ``enumerate_hubs() -> list[str]``:
  Scan USB serial ports; return device paths whose serial number starts with
  ``"D"`` (REV Hub convention).

- ``connect(port=None, ...) -> Hub``:
  Open a ``SerialTransport``, build a ``Session``, broadcast ``Discovery`` to
  ``0xFF``, identify the parent module, call ``QueryInterface("DEKA")`` to
  resolve and store ``session.deka_base``, then construct and return the parent
  ``Hub``.  If *port* is ``None``, ``enumerate_hubs()`` is called to find the
  first available port.

- ``discover(session, dest=0xFF) -> list[RawPacket]``:
  Send one ``Discovery`` frame and accumulate ``Discovery_RSP`` packets until a
  50 ms quiet window elapses.  No ``time.sleep`` with argument ≥ 0.1 s.

Notes
-----
``discover()`` is exempt from ``ref_num`` correlation — the broadcast may elicit
replies from multiple modules, each with its own source address.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from rhsp.errors import RhspError
from rhsp.framing import FrameParser, RawPacket
from rhsp.session import Session
from rhsp.transport import SerialTransport

if TYPE_CHECKING:
    pass

__all__ = ["enumerate_hubs", "connect", "discover"]

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

_DISCOVERY_RSP_TYPE: int = 0xFF0F

# Quiet-window duration for discover() — seconds of silence that signals end of replies.
_QUIET_WINDOW: float = 0.050

# Bytes to read per transport poll.
_READ_CHUNK: int = 512


# ---------------------------------------------------------------------------
# enumerate_hubs
# ---------------------------------------------------------------------------


def enumerate_hubs() -> list[str]:
    """Return serial port device paths whose USB serial number starts with ``"D"``.

    Uses ``serial.tools.list_ports`` to scan all available ports.  Only ports
    whose ``serial_number`` attribute begins with the letter ``"D"`` are
    returned — this matches REV Robotics Hub USB serial numbers (e.g.
    ``"DQ3M375O"``).

    Returns
    -------
    list[str]
        Ordered list of port device paths (e.g. ``["/dev/cu.usbserial-DQ3M375O"]``).
        Empty list if no matching port is found.

    Notes
    -----
    No hub connection is made; the function only queries the OS port list.
    """
    try:
        from serial.tools import list_ports  # type: ignore[import-untyped]
    except ImportError as exc:  # pragma: no cover
        raise RhspError("pyserial is required for enumerate_hubs()") from exc

    return [
        port.device
        for port in list_ports.comports()
        if port.serial_number and str(port.serial_number).startswith("D")
    ]


# ---------------------------------------------------------------------------
# discover
# ---------------------------------------------------------------------------


def discover(session: Session, dest: int = 0xFF) -> list[RawPacket]:
    """Send a Discovery broadcast and collect all ``Discovery_RSP`` replies.

    Sends one ``Discovery`` frame to *dest* (typically the broadcast address
    ``0xFF``) then accumulates incoming ``Discovery_RSP`` packets until no new
    bytes arrive for approximately 50 ms (the quiet window).

    This function does **not** use ``time.sleep()`` with an argument ≥ 0.1 s.
    The quiet window is implemented by measuring elapsed time between
    ``transport.read()`` calls returning empty bytes.

    Parameters
    ----------
    session:
        An open :class:`~rhsp.session.Session`.
    dest:
        Destination address for the Discovery frame (default ``0xFF``).

    Returns
    -------
    list[RawPacket]
        All ``Discovery_RSP`` packets received within the quiet window.
        Not subject to ``ref_num`` correlation.
    """
    # Use the session's internal transport and advance its message counter so
    # framing stays consistent.  We need low-level access to send the frame and
    # read the raw reply stream, so we delegate the send to session.discover()
    # which already implements the quiet-window drain idiom.
    return session.discover(dest=dest)


# ---------------------------------------------------------------------------
# connect
# ---------------------------------------------------------------------------


def connect(
    port: str | None = None,
    *,
    baud: int = 460_800,
    timeout: float = 1.0,
    retries: int = 3,
) -> "Hub":
    """Open a connection to a REV Hub and return a :class:`Hub` object.

    Steps performed:

    1. If *port* is ``None``, call :func:`enumerate_hubs` to find the first
       available port.  Raise :exc:`~rhsp.errors.RhspError` if none found.
    2. Open a :class:`~rhsp.transport.SerialTransport` at *baud* baud.
    3. Build a :class:`~rhsp.session.Session`.
    4. Call :func:`discover` to broadcast ``Discovery`` to ``0xFF`` and
       collect all ``Discovery_RSP`` replies.
    5. Identify the parent hub — the reply whose ``parent`` payload field is
       non-zero, or the one with the lowest address if none is marked parent.
    6. Call ``session.query_interface(parent_address, "DEKA")`` to resolve the
       DEKA base packet ID and store it in ``session.deka_base``.
    7. Construct and return the parent :class:`Hub`.  Child hub addresses are
       stored on the parent as ``Hub.children``.

    Parameters
    ----------
    port:
        OS device path (e.g. ``"/dev/cu.usbserial-DQ3M375O"``).  When
        ``None``, the first port returned by :func:`enumerate_hubs` is used.
    baud:
        Baud rate for the serial port (default 460 800).
    timeout:
        Session transaction timeout in seconds.
    retries:
        Number of additional retry attempts per transaction.

    Returns
    -------
    Hub
        Connected parent hub with ``session``, ``address``, ``children``, and
        ``deka_base`` populated.

    Raises
    ------
    RhspError
        If no hub port is found when *port* is ``None``.
    RhspTimeoutError
        If Discovery or QueryInterface times out.
    """
    if port is None:
        ports = enumerate_hubs()
        if not ports:
            raise RhspError("No hub found")
        port = ports[0]

    transport = SerialTransport(port, baud=baud, timeout=timeout)
    session = Session(transport, retries=retries, timeout=timeout)

    # Broadcast discovery and collect all replies.
    packets = discover(session, dest=0xFF)

    # Parse Discovery_RSP payloads to determine hub addresses and parent flag.
    # The Discovery_RSP source address is the hub's module address.
    # The single payload byte is the ``parent`` flag (non-zero = parent hub).
    hub_infos: list[tuple[int, bool]] = []
    for pkt in packets:
        parent_flag = bool(pkt.payload[0]) if pkt.payload else False
        hub_infos.append((pkt.src, parent_flag))

    # If no replies, we have no hub to talk to.
    if not hub_infos:
        raise RhspError("Discovery returned no replies — is the hub powered on?")

    # Identify the parent address.
    parent_address: int | None = None
    child_addresses: list[int] = []
    for addr, is_parent in hub_infos:
        if is_parent:
            parent_address = addr
        else:
            child_addresses.append(addr)

    if parent_address is None:
        # Fallback: if no hub marked itself as parent, pick the one that replied
        # first (typically address 2 on physical hardware).
        parent_address = hub_infos[0][0]
        child_addresses = [a for a, _ in hub_infos[1:]]

    # Resolve DEKA base via QueryInterface.
    session.query_interface(parent_address, "DEKA")
    # session.deka_base is now updated by query_interface().

    # Construct and return the Hub object.
    from rhsp.hub import Hub  # imported here to avoid circular dependency

    parent_hub = Hub(session=session, address=parent_address)
    parent_hub.children = [
        Hub(session=session, address=addr) for addr in child_addresses
    ]
    return parent_hub
