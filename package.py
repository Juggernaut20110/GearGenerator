"""Compile the window into a single distributable GearGenerator.exe, with Nuitka.

    .venv-build\\Scripts\\python.exe package.py
    .venv-build\\Scripts\\python.exe package.py --debug

The interpreter is the *build* venv one, not `.venv`. Nuitka compiles with the
interpreter it runs under and bundles that interpreter's stdlib and site
packages, so the build environment is part of what ships - which is exactly why
it is kept apart from the runtime one. `.venv` stays what `requirements.txt`
says it is, and `python -m pytest` there keeps measuring the shipped tree.

Only the window is packaged. It already reaches all five builders through its
Build button, and driving them from the GUI rather than from `tools/` is the
better-exercised path - it is what found the last four bugs.

**What the exe still needs from the machine it runs on: nothing, until the
Build button.** The geometry, the preview, the report and the DXF export are
pure Python and travel whole. Building actual solids needs SOLIDWORKS installed
there, the same as it does from a checkout.

Three properties of this tree are what make the flag list below so short, and
each of them is worth knowing before adding to it:

* **There are no data files.** Nothing under `gears/` opens a bundled asset -
  the only file reads are the user's own preset JSON, chosen from a dialog. So
  there is nothing to `--include-data-files`.
* **Every import is static.** `gears/sw/__init__.py` imports all five builders
  eagerly, so the `getattr(sw, builder)` the GUI dispatches through resolves
  against a module the compiler has already followed. No `--include-package` is
  needed to find it.
* **The COM layer is late-binding only.** `sw/session.py` uses `Dispatch` and
  never `gencache`/`EnsureDispatch` - forced on it because `EnsureDispatch`
  cannot introspect `ISldWorks`. A generated `gen_py` cache is the usual way a
  frozen COM application dies, and this codebase never had one to lose.

If something does come up missing, add the flag *and the measurement that
demanded it*. A speculative `--include-package` hides the thing worth knowing.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ENTRY = ROOT / "run.py"
OUTPUT_DIR = ROOT / "build"
EXE_NAME = "GearGenerator.exe"

COMPANY = "Carl Rule"
# The display name, which is why it carries a space where the file name does
# not. It reaches the exe's Properties pane and, through the onefile tempdir
# spec below, the name of the cache directory.
PRODUCT = "Gear Generator"
VERSION = "1.0.0"
DESCRIPTION = "Bevel, spur and planetary gear generator for SOLIDWORKS"


def nuitka_command(debug: bool) -> list[str]:
    """The flag list, with the reason for each flag that has one."""
    command = [
        sys.executable, "-m", "nuitka",

        # Standalone bundles the interpreter and the stdlib; onefile then packs
        # that bundle into the single file we actually want to hand someone.
        "--standalone",
        "--onefile",

        # The window. Pulls Tcl/Tk 8.6 out of the base installation, which on
        # this machine is a python.org per-user install with tcl/ beside it -
        # the layout the plugin expects. A Microsoft Store Python does not have
        # it there, and that is the one environment change that would break
        # this build.
        "--enable-plugin=tk-inter",

        # No console window flashing up behind the tkinter one. The cost is
        # that anything failing *before* the window exists fails silently,
        # which is what --debug below exists to undo.
        "--windows-console-mode=disable",

        # A onefile exe unpacks its payload before running. The default spec
        # picks a fresh %TEMP% directory every launch, and with Tcl/Tk in the
        # payload that is tens of megabytes of extraction each time the user
        # opens the program. Pinning it to a cache directory makes the first
        # launch pay it once. {VERSION} is what stops a new build silently
        # reusing the payload an old one left behind. Nuitka infers the cache
        # mode from the spec rather than taking it as a separate flag: a spec
        # with no run-dependent component in it, like this one, means cached.
        "--onefile-tempdir-spec={CACHE_DIR}/{COMPANY}/{PRODUCT}/{VERSION}",

        # Nuitka needs a C compiler and will fetch a MinGW64 toolchain if it
        # cannot find one. Taking the offer up front keeps the build
        # non-interactive; where MSVC is installed it is preferred and nothing
        # is downloaded.
        "--assume-yes-for-downloads",

        # Right-click -> Properties on the exe shows these.
        f"--company-name={COMPANY}",
        f"--product-name={PRODUCT}",
        f"--file-version={VERSION}",
        f"--product-version={VERSION}",
        f"--file-description={DESCRIPTION}",

        # None of these is reachable from `gears`, so this only guards against
        # a stray future import dragging the test framework into the exe.
        "--nofollow-import-to=pytest,tests,tools",

        f"--output-dir={OUTPUT_DIR}",
        f"--output-filename={EXE_NAME}",
    ]

    if debug:
        # A console to read a traceback in, and the intermediate .dist folder
        # kept so a missing DLL can be looked for by name.
        command.remove("--windows-console-mode=disable")
        command.append("--windows-console-mode=force")
    else:
        # Drops the generated C and the unpacked .dist once the exe is built.
        command.append("--remove-output")

    command.append(str(ENTRY))
    return command


def check_environment() -> None:
    """Fail early and say which interpreter to use instead.

    Running this from `.venv` gets a long way in before Nuitka complains, and
    the resulting error is about a missing module rather than about the wrong
    interpreter - so it is worth catching here where the message can be plain.
    """
    try:
        import nuitka  # noqa: F401
    except ImportError:
        sys.exit(
            "Nuitka is not installed in this interpreter.\n"
            f"  running: {sys.executable}\n"
            "  expected: .venv-build\\Scripts\\python.exe\n\n"
            "Create it with:\n"
            "  python -m venv .venv-build\n"
            "  .venv-build\\Scripts\\pip install -r requirements-build.txt"
        )

    try:
        import win32com.client  # noqa: F401
    except ImportError:
        sys.exit(
            "pywin32 is missing from the build environment. It is compiled\n"
            "into the exe, so without it here the Build button ships broken.\n"
            "  .venv-build\\Scripts\\pip install -r requirements-build.txt"
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--debug",
        action="store_true",
        help="build with a console attached and keep the unpacked .dist folder,"
             " which is the only way to read a crash that happens before the"
             " window appears",
    )
    args = parser.parse_args(argv)

    check_environment()

    command = nuitka_command(args.debug)
    print(" ".join(command), "\n")
    result = subprocess.run(command, cwd=ROOT)
    if result.returncode != 0:
        return result.returncode

    exe = OUTPUT_DIR / EXE_NAME
    if not exe.exists():
        print(f"\nNuitka reported success but {exe} is not there.")
        return 1
    print(f"\n{exe}  {exe.stat().st_size / 1024 / 1024:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
