"""Optional mDNS lookup of a robot by id — native only.

Never imported from the Pyodide branch of :mod:`bonicos.robot` at all (that
branch binds the host-preloaded WebRTC transport instead), so this module
doesn't need the lazy-import dance ``transports/websocket.py`` and
``transports/webrtc.py`` require — ``zeroconf`` is simply an optional
``native`` extra, imported inside the one function that needs it.

**Assumption, pending alignment with the robot-side team:** the mDNS service
type/TXT record shape below (``_bonicos._tcp.local.``, a ``robotId`` TXT
property) isn't pinned by any of the four `bonicOS-SDK` spec docs — it's
this build's proposed convention for whatever advertises the service on the
robot/tablet side.
"""

from __future__ import annotations

import time
from typing import Optional

SERVICE_TYPE = "_bonicos._tcp.local."


def find_robot(robot_id: Optional[str] = None, timeout: float = 5.0) -> Optional[str]:
    """Return the IP of a robot advertising ``SERVICE_TYPE`` over mDNS.

    If ``robot_id`` is given, only a service whose TXT record's ``robotId``
    matches is returned. Returns ``None`` if nothing is found within
    ``timeout`` seconds.
    """
    from zeroconf import ServiceBrowser, Zeroconf

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
