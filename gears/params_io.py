"""Preset save/load, shared by every gear type's parameter dataclass.

The unknown-key filter in `from_json` is what makes presets tolerant in both
directions: a file written before a field existed still loads, and a file
written by a later version does not crash an older one. Adding a field to a
params dataclass therefore costs nothing here.
"""

from __future__ import annotations

import json
from dataclasses import asdict, fields
from pathlib import Path


class JsonParams:
    """Mixin for a frozen params dataclass: write it out, read it back."""

    def to_json(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")

    @classmethod
    def from_json(cls, path: str | Path):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in known})
