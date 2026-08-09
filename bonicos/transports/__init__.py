"""Transport backends. Nothing heavy is imported here — each backend module
lazy-imports its own runtime (``websockets``, or ``js``/``pyodide.ffi``)
inside ``connect()`` so importing this package never pulls in a dependency
absent from the current environment (ARCHITECTURE.md §1).
"""
