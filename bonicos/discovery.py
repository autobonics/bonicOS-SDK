"""Optional mDNS lookup of a robot by id.

Only reached when :class:`~bonicos.robot.BonicBot` is constructed with no
``host``. ``zeroconf`` is an optional ``discovery`` extra, imported inside
the one function that needs it so a plain install never pays for it.

**Assumption, pending alignment with the robot-side team:** the mDNS service
type/TXT record shape below (``_bonicos._tcp.local.``, a ``robotId`` TXT
property) isn't pinned by any of the four `bonicOS-SDK` spec docs — it's
this build's proposed convention for whatever advertises the service on the
robot/tablet side.
"""

from __future__ import annotations

import time
from typing import Optional

from .exceptions import ConnectionError as BonicConnectionError

SERVICE_TYPE = "_bonicos._tcp.local."


def find_robot(robot_id: Optional[str] = None, timeout: float = 5.0) -> Optional[str]:
    """Return the IP of a robot advertising ``SERVICE_TYPE`` over mDNS.

    If ``robot_id`` is given, only a service whose TXT record's ``robotId``
    matches is returned. Returns ``None`` if nothing is found within
    ``timeout`` seconds.

    Raises :class:`~bonicos.exceptions.ConnectionError` if the optional
    ``discovery`` extra isn't installed — reaching here at all means the
    caller built ``BonicBot()`` with no ``host``, so an actionable message
    beats the bare ``ModuleNotFoundError`` zeroconf would otherwise raise.
    """
    try:
        from zeroconf import ServiceBrowser, Zeroconf
    except ImportError as exc:
        raise BonicConnectionError(
            "mDNS discovery needs the optional dependency: "
            "`pip install bonicos[discovery]` — or pass the robot's address "
            'directly, e.g. BonicBot("192.168.1.50", robot_id="M1_001")'
        ) from exc

    found: dict = {}

    class _Listener:
        def add_service(self, zc: Zeroconf, service_type: str, name: str) -> None:
            info = zc.get_service_info(service_type, name)
            if info is None or not info.addresses:
                return
            properties = {
                k.decode() if isinstance(k, bytes) else k: (
                    v.decode() if isinstance(v, bytes) else v
                )
                for k, v in (info.properties or {}).items()
            }
            if robot_id is not None and properties.get("robotId") != robot_id:
                return
            found["host"] = ".".join(str(b) for b in info.addresses[0])

        def update_service(self, *args: object, **kwargs: object) -> None:
            pass

        def remove_service(self, *args: object, **kwargs: object) -> None:
            pass

    zeroconf = Zeroconf()
    try:
        ServiceBrowser(zeroconf, SERVICE_TYPE, _Listener())
        deadline = time.monotonic() + timeout
        while "host" not in found and time.monotonic() < deadline:
            time.sleep(0.1)
        return found.get("host")
    finally:
        zeroconf.close()
