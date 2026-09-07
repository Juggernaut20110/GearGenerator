"""Gear generators: geometry, validation, and SOLIDWORKS construction.

One sub-package per gear type - `gears.bevel`, `gears.spur`, `gears.hypoid` - each holding its
own params, geometry, validator, mesh arithmetic and preview scenes. What the
types share lives at this level: the planar involute core, the placement
arithmetic, the drawing primitives, and the whole of `gears.sw`.

Nothing is imported eagerly here. `gears.gui` needs tkinter and `gears.sw` needs
pywin32, so both stay opt-in - `import gears.spur` costs neither.
"""
