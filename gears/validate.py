"""Validation primitives shared by every gear type. No COM, no GUI.

Errors block a build; warnings let it proceed but say what is unusual. The
split matters because plenty of perfectly buildable gear sets are outside
textbook proportions, and the tool should not refuse to draw them.

A validator is plain imperative code, not a registry of rules. It never raises -
it reports everything it can find in one pass, so a user fixing four things at
once sees all four.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Practical bounds shared by the types. Deliberately generous - these are sanity
# rails, not a design standard.
MIN_TEETH = 6
MIN_PRESSURE_ANGLE = 14.5
MAX_PRESSURE_ANGLE = 25.0


@dataclass(frozen=True)
class Issue:
    field: str
    message: str

    def __str__(self) -> str:
        return f"{self.field}: {self.message}"


@dataclass
class ValidationResult:
    errors: list[Issue] = field(default_factory=list)
    warnings: list[Issue] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    def error(self, field_: str, message: str) -> None:
        self.errors.append(Issue(field_, message))

    def warn(self, field_: str, message: str) -> None:
        self.warnings.append(Issue(field_, message))
