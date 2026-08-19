"""The shape of the terminal report, shared by every gear type.

Three columns: a label, then the pinion and gear values, then a unit. Column
widths are fixed rather than computed, so a bevel report and a spur report line
up against each other when you put them side by side.
"""

from __future__ import annotations

import sys


def row(label: str, pinion, gear="", unit="") -> str:
    return f"  {label:<28}{pinion:>14}{gear:>14}  {unit}"


def emit_issues(result) -> bool:
    """Print a validation result to stderr. True if the build may proceed.

    Errors and warnings both go to stderr so that piping the report somewhere
    keeps the numbers clean and still shows the complaints on the terminal.
    """
    for issue in result.errors:
        print(f"ERROR    {issue}", file=sys.stderr)
    for issue in result.warnings:
        print(f"WARNING  {issue}", file=sys.stderr)
    if not result.ok:
        return False
    if result.warnings:
        print(file=sys.stderr)
    return True
