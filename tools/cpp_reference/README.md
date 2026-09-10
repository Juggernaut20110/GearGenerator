# C++ reference snapshots

`export.py` runs the Python implementation and writes compact JSON fixtures for
the native regression test.  The JSON is a development artifact: it is not a
runtime input format and intentionally contains scalar derived values instead
of large tooth-space point arrays.

From the repository root:

```powershell
.\.venv\Scripts\python.exe tools\cpp_reference\export.py
```

The native test reads `cpp/tests/fixtures/python_reference.json`.  Re-export
the file whenever the Python reference changes, review the diff, and only then
update native behavior.  Floating-point fields are compared numerically using
the tolerance documented in `cpp/src/core/common/numerics.hpp`.
