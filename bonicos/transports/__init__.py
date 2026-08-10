"""Transport backends.

``websocket.py`` is the only transport to a real robot; ``mock.py`` is the
hardware-free double used by the tests. ``base.py`` defines the interface
:mod:`bonicos.robot` depends on, and is the seam that lets those two be
interchangeable.

Nothing heavy is imported here — ``websocket.py`` lazy-imports
``websockets`` inside ``connect()`` and ``_camera_link.py`` lazy-imports
``aiortc``/``numpy`` (the optional ``camera`` extra), so ``import bonicos``
stays cheap.
"""
