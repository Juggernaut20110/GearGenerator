"""Launch the gear generator.

    .venv\\Scripts\\python.exe run.py
    .venv\\Scripts\\python.exe run.py presets\\anchor.json

An optional argument is a parameter preset saved from the GUI. For the
terminal report instead of the window, use `python -m gears --help`.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gears.gui import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
