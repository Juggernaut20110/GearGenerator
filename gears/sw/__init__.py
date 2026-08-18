"""SOLIDWORKS COM driver. Everything that touches pywin32 lives under here."""

from .session import SwError, SwSession, connect
from .bevel_part import BuildResult, build_gear
from .bevel_assembly import SetResult, build_set

__all__ = [
    "BuildResult",
    "SetResult",
    "SwError",
    "SwSession",
    "build_gear",
    "build_set",
    "connect",
]
