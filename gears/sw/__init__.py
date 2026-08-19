"""SOLIDWORKS COM driver. Everything that touches pywin32 lives under here.

`session` is the API discipline and is shared by everything. `common` and
`assembly_common` hold the build steps that do not depend on the gear type;
each type then has an assembly builder of its own.

There are three assembly builders and only two part builders, which is not an
omission. A planetary train is made of spur gears - an external sun, external
planets and an internal ring - so `spur_part` builds all three of its parts and
only the *assembly* has anything new in it.
"""

from .session import SwError, SwSession, connect
from .common import BuildResult
from .bevel_part import build_gear
from .bevel_assembly import SetResult, build_set
from .spur_part import build_spur
from .spur_assembly import SpurSetResult, build_spur_set
from .planetary_assembly import PlanetarySetResult, build_planetary_set

__all__ = [
    "BuildResult",
    "PlanetarySetResult",
    "SetResult",
    "SpurSetResult",
    "SwError",
    "SwSession",
    "build_gear",
    "build_planetary_set",
    "build_set",
    "build_spur",
    "build_spur_set",
    "connect",
]
