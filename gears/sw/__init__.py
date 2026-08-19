"""SOLIDWORKS COM driver. Everything that touches pywin32 lives under here.

`session` is the API discipline and is shared by everything. `common` and
`assembly_common` hold the build steps that do not depend on the gear type;
each type then has a part builder and an assembly builder of its own.
"""

from .session import SwError, SwSession, connect
from .common import BuildResult
from .bevel_part import build_gear
from .bevel_assembly import SetResult, build_set
from .spur_part import build_spur
from .spur_assembly import SpurSetResult, build_spur_set

__all__ = [
    "BuildResult",
    "SetResult",
    "SpurSetResult",
    "SwError",
    "SwSession",
    "build_gear",
    "build_set",
    "build_spur",
    "build_spur_set",
    "connect",
]
