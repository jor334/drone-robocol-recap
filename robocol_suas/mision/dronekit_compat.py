"""DroneKit en Python moderno.

DroneKit 2.9.2 (la última versión, de 2019) usa `collections.MutableMapping`, que Python 3.10 movió a
`collections.abc`. Sin este parche, `import dronekit` falla con AttributeError.

Se importa siempre desde aquí:
    from robocol_suas.mision.dronekit_compat import connect, VehicleMode, LocationGlobalRelative
"""
import collections
import collections.abc

if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import *  # noqa: E402,F401,F403
