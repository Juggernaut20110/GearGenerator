"""Does the geometry survive compilation? Ask it, rather than assuming it.

    .venv\\Scripts\\python.exe tools\\probe_compiled_geometry.py
    .venv-build\\Scripts\\python.exe tools\\probe_compiled_geometry.py --compare

`package.py` hands the window to people who have no Python, which puts a C
compiler and Nuitka's optimiser between what the tests measure and what those
people run. Nothing in `tests/` can see across that gap: the suite exercises the
source tree, and the exe is a different artifact built from it.

One question, asked the way the rest of `tools/` asks them - in isolation, so
that when a build looks wrong this is not also the thing under test:

**Does compiled arithmetic agree with interpreted arithmetic, exactly?**

Run plainly, this prints the terminal report for every anchor set in the README.
`--compare` compiles *this file* with Nuitka, runs the binary, and diffs its
output against the interpreted run. Equality has to be byte-for-byte. A gear is
a fit between two surfaces and the reports carry four decimal places, so there
is no tolerance to argue about here - either the compiler preserved the
floating-point work or it did not.

Measured on Nuitka 4.1.3 / Python 3.13.7 / MSVC 14.3: identical.

Two things this deliberately does *not* do. It does not touch `gears/sw/`, which
needs a CAD seat and cannot run on a build machine at all. And it does not build
the window - the exe `package.py` produces is a different entry point, so a pass
here says the geometry travelled, not that the program starts.
"""

from __future__ import annotations

import argparse
import difflib
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from gears.__main__ import main as report_main            # noqa: E402

# The README's anchor sets, chosen to sit beside each other. Every branch the
# geometry has is represented: a straight bevel and both curved ones, an
# external spur pair straight and helical, an internal pair, and a train.
ANCHORS = [
    ["--module", "2", "--z1", "17", "--z2", "43"],
    ["--module", "2", "--z1", "17", "--z2", "43", "--spiral", "35"],
    ["--module", "2", "--z1", "17", "--z2", "43", "--zerol"],
    ["--type", "spur", "--module", "2", "--z1", "17", "--z2", "43"],
    ["--type", "spur", "--module", "2", "--z1", "17", "--z2", "43", "--beta", "15"],
    ["--type", "spur", "--module", "2", "--z1", "18", "--z2", "60", "--internal"],
    ["--type", "planetary", "--module", "2", "--z1", "24", "--z2", "18"],
    ["--type", "hypoid", "--module", str(170 / 42), "--z1", "13", "--z2", "42",
     "--offset", "15", "--face-width", "30", "--spiral", "50",
     "--cutter-radius", "63.5"],
]


def print_reports() -> None:
    """Every anchor set's report, on stdout, with nothing else mixed in.

    The exit status is printed alongside because it is part of the answer: a
    set that stopped validating would otherwise change nothing visible here.
    """
    for argv in ANCHORS:
        print("=" * 72)
        print(" ".join(argv))
        print("=" * 72)
        status = report_main(argv)
        print(f"exit={status}")


def _run(command: list[str], env: dict[str, str] | None = None) -> str:
    result = subprocess.run(
        command, cwd=ROOT, env=env, capture_output=True, text=True
    )
    if result.returncode != 0:
        sys.exit(
            f"{command[0]} failed with {result.returncode}\n"
            f"{result.stdout}\n{result.stderr}"
        )
    return result.stdout


def compare() -> int:
    """Compile this file, run it, and diff the two outputs."""
    try:
        import nuitka  # noqa: F401
    except ImportError:
        sys.exit(
            "Nuitka is not installed in this interpreter.\n"
            f"  running: {sys.executable}\n"
            "  expected: .venv-build\\Scripts\\python.exe"
        )

    interpreted = _run([sys.executable, str(Path(__file__).resolve())])
    print(f"interpreted  {len(interpreted.splitlines())} lines")

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as work:
        # Nuitka resolves imports against the compiling process's path, and
        # this file lives in tools/ - so without the repo root on PYTHONPATH
        # the `gears` import fails at compile time rather than at run time.
        env = dict(os.environ, PYTHONPATH=str(ROOT))
        _run(
            [
                sys.executable, "-m", "nuitka",
                "--standalone",              # onefile would only add pack time
                "--assume-yes-for-downloads",
                f"--output-dir={work}",
                str(Path(__file__).resolve()),
            ],
            env=env,
        )
        exe = Path(work) / "probe_compiled_geometry.dist" / "probe_compiled_geometry.exe"
        if not exe.exists():
            sys.exit(f"Nuitka reported success but {exe} is not there.")
        compiled = _run([str(exe)])

    print(f"compiled     {len(compiled.splitlines())} lines")

    if interpreted == compiled:
        print(f"\nIDENTICAL, byte for byte, across {len(ANCHORS)} anchor sets.")
        return 0

    print("\nDIVERGED. The compiler did not preserve the arithmetic:\n")
    for line in difflib.unified_diff(
        interpreted.splitlines(), compiled.splitlines(),
        fromfile="interpreted", tofile="compiled", lineterm="", n=2,
    ):
        print(line)
    return 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--compare",
        action="store_true",
        help="compile this file with Nuitka and diff the compiled output"
             " against the interpreted one; needs the build venv",
    )
    args = parser.parse_args(argv)

    if args.compare:
        return compare()
    print_reports()
    return 0


if __name__ == "__main__":
    sys.exit(main())
