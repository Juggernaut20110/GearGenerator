# Gear Generator

Generates gear sets as real SOLIDWORKS parts and meshed assemblies, from a
handful of numbers. Pure-Python geometry engine, a tkinter front end, and a COM
driver that builds the solids.

Four gear types, sharing everything they can:

* **bevel**, straight, spiral or **Zerol** — module, tooth counts, pressure
  angle, shaft angle, mean spiral angle and hand, cutter radius, face width,
  bore, hub, root rim and backlash
* **involute spur**, straight or helical, **external or internal** — normal
  module, tooth counts, normal pressure angle, helix angle and hand, face
  width, bore, hub, backlash, and a rim for a ring gear
* **planetary** — a sun, N planets and an internal ring on one set of numbers:
  normal module, sun and planet tooth counts, how many planets, backlash
* **hypoid**, Gleason/ISO Method 1 — a skew-axis external pair: outer transverse
  module, tooth counts, shaft angle, signed axis offset, face width, pinion
  spiral angle, cutter radius, pressure angle, hand, bore, hub, root rim and
  backlash

Every one produces:

* a fully dimensioned blank sketch, revolved, with every dimension driven by a
  named global variable you can edit in SOLIDWORKS afterwards
* true involute teeth for spur/bevel/internal types; the hypoid builder uses
  approximate Tredgold/back-cone tooth surfaces, lofted between 3D-sketch
  sections and circular patterned
* every member plus an assembly correctly clocked and meshing, mated so it
  articulates: gear mates couple the members, and dragging one turns the rest
  in the right ratio

The anchor cases used throughout the code and tests, chosen to sit beside each
other:

```
bevel        m=2, 17x43, 20 deg pressure angle, 90 deg shafts
             plus 35 deg mean spiral for the spiral set
             plus a 39.3035 mm cutter at 0 deg for the Zerol set
spur         m=2, 17x43, 20 deg, plus a 15 deg helix for the helical set
internal     m=2, 18x60 - which is also the planetary set's planet-ring mesh
planetary    m=2, sun 24, planet 18, ring 60, 3 planets
hypoid       m_o=170/42, 13x42, 20 deg, 90 deg shafts, 15 mm offset,
             50 deg pinion spiral, 63.5 mm cutter (ISO Method 1 sample)
```

---

## Running it

The interpreter is the venv one. Always. There is no global install.

```
.venv\Scripts\python.exe run.py                      # the GUI, all four types
.venv\Scripts\python.exe -m gears --module 2 --z1 17 --z2 43
.venv\Scripts\python.exe -m gears --module 2 --z1 17 --z2 43 --spiral 35
.venv\Scripts\python.exe -m gears --module 2 --z1 17 --z2 43 --zerol
.venv\Scripts\python.exe -m gears --type spur --module 2 --z1 17 --z2 43 --beta 15
.venv\Scripts\python.exe -m gears --type spur --module 2 --z1 18 --z2 60 --internal
.venv\Scripts\python.exe -m gears --type planetary --module 2 --z1 24 --z2 18
.venv\Scripts\python.exe -m gears --type hypoid --module 4.047619 --z1 13 --z2 42 --offset 15 --face-width 30 --spiral 50 --cutter-radius 63.5
.venv\Scripts\python.exe -m pytest -q                # 1001 collected, no SOLIDWORKS
```

`--type` defaults to `bevel`, which is what the tool generated before there was
a choice. A flag belonging to another type is refused rather than ignored, so
`--sigma` on a spur set is an error and not a silent no-op — while a flag two
types genuinely share, like `--beta`, is refused only by the third, and one all
three share, like `--backlash`, is refused by none. A flag is *created* by
exactly one type's `add_arguments` — every one of them runs against the same
parser — and *claimed* by each type that means the same thing by it.

For a planetary set `--z1` and `--z2` are the sun and the planet, and `--z-sun`
and `--z-planet` say the same thing more plainly. **The ring is derived**, so it
has no flag: `z_ring = z_sun + 2 z_planet` is forced by the two centre distances
having to agree.

Building actual geometry needs SOLIDWORKS running (or installed — the session
will start it):

```
.venv\Scripts\python.exe tools\build_gear.py --member gear
.venv\Scripts\python.exe tools\build_gear.py --spiral 35
.venv\Scripts\python.exe tools\build_gear.py --zerol
.venv\Scripts\python.exe tools\build_set.py --z1 17 --z2 43 --spiral 35
.venv\Scripts\python.exe tools\build_spur.py --member gear --beta 15
.venv\Scripts\python.exe tools\build_spur_set.py --z1 17 --z2 43
.venv\Scripts\python.exe tools\build_spur_set.py --z1 18 --z2 60 --internal
.venv\Scripts\python.exe tools\build_planetary.py --z-sun 24 --z-planet 18
.venv\Scripts\python.exe tools\build_hypoid_set.py --module 4.047619 --z1 13 --z2 42 --offset 15 --face-width 30 --spiral 50 --cutter-radius 63.5
```

`--help` on any of those lists the parameter flags. Output lands in `out/`,
which is gitignored along with all SOLIDWORKS file types.

---

## Packaging

`package.py` compiles the window into one `GearGenerator.exe` with Nuitka, for
handing to someone who has no Python and no checkout.

```
python -m venv .venv-build
.venv-build\Scripts\pip install -r requirements-build.txt
.venv-build\Scripts\python.exe package.py            # build\GearGenerator.exe
.venv-build\Scripts\python.exe package.py --debug    # console attached, .dist kept
```

**The interpreter is the build venv one, and that is the point of there being
two.** Nuitka compiles with the interpreter it runs under and bundles that
interpreter's stdlib and site packages, so the build environment is part of what
ships. Keeping it out of `.venv` is what lets `.venv` stay exactly what
`requirements.txt` says it is, so `python -m pytest` there keeps measuring the
shipped tree and nothing else.

Measured on this machine, Nuitka 4.1.3 / Python 3.13.7 / MSVC 14.3:

```
exe                7.6 MB      27.4 MB payload compressed to 28.4 %
cold start    1.2 - 1.4 s      unpacks Tcl/Tk to the cache directory
warm start    0.5 - 0.6 s      cache hit, no unpacking
cache              26.1 MB     %LOCALAPPDATA%\Carl Rule\Gear Generator\0.1.0\
build              ~2 min      from cold; clcache makes the second one faster
```

**Only the window is packaged.** It reaches all seven builders through its Build
button, and it is the better-exercised path — driving the builders from the GUI
rather than from `tools/` is what found the four bugs recorded below.

**The exe needs nothing from the machine it runs on until the Build button.**
The geometry, the preview, the report and the DXF export are pure Python and
travel whole. Building solids needs SOLIDWORKS installed there, exactly as it
does from a checkout.

### Why this tree freezes cleanly

Three things that usually break a frozen Python program are absent here, and all
three are consequences of decisions made for other reasons:

* **There are no data files.** Nothing under `gears/` opens a bundled asset. The
  only file reads are the user's own preset JSON, chosen from a dialog.
* **Every import is static.** `sw/__init__.py` imports all seven builders
  eagerly, so the `getattr(sw, builder)` the GUI dispatches through resolves
  against a module the compiler has already followed.
* **The COM layer is late-binding only.** `session.py` uses `Dispatch` and never
  `gencache`/`EnsureDispatch` — forced on it because `EnsureDispatch` cannot
  introspect `ISldWorks`. A generated `gen_py` cache is the usual way a frozen
  COM application dies, and this codebase never had one to lose.

So the flag list in `package.py` carries no `--include-data-files` and no
`--include-package`. If something turns up missing, add the flag *and the
measurement that demanded it*; a speculative include hides the thing worth
knowing.

### What was measured, and what is still Carl's to check

Two questions are cheaper to settle against a compiled binary than to discover
in the window, and both were asked before shipping one:

```
COM survives compilation   pythoncom313.dll and pywintypes313.dll are bundled
                           unasked; VARIANT construction, all seven builders and
                           SwSession all resolve.  No SOLIDWORKS needed - the
                           DLL load happens at import
geometry survives it       all eight anchor sets' terminal reports, compiled
                           against interpreted: byte-for-byte identical,
                           451 lines
```

The second is a standing check rather than a one-off, because the thing it
guards against arrives with a dependency upgrade rather than with an edit:
[tools/probe_compiled_geometry.py](tools/probe_compiled_geometry.py) `--compare`
compiles itself, runs the binary and diffs it against the interpreted run.
`tests/` measures the source tree and the exe is a different artifact built from
it, so nothing in the suite can see across that gap. The first was a throwaway;
the DLL either travels or the program cannot reach SOLIDWORKS at all, which is
not a subtle failure.

**`sys.frozen` is False under Nuitka**, which is worth knowing because it is the
attribute everyone reaches for first. `__compiled__` is the global Nuitka
injects, and it is what `gui.py` reads to decide which remedy to offer when the
COM import fails — a checkout has a venv to install pywin32 into and the exe does
not, so offering the wrong one is worse than offering none.

Also worth knowing when timing a launch: **onefile runs a bootstrap parent that
spawns the real process**, and the window belongs to the child. Watching the
parent for a window title waits forever.

What no probe here settles is the Build button from inside the frozen exe
against a live SOLIDWORKS. That is one for a hand on the machine, and it ranks
above everything above it.

**The exe is unsigned**, so SmartScreen will warn on first run on any machine but
the one that built it. That is a certificate, not a build fault.

### Versioning

`package.py` carries `VERSION = "0.1.0"`, and a **tagged** CI build overrides it
from the tag — so the constant is the development version and the tag is the
released one. Tag `v0.2.0` and the exe says 0.2.0 without anyone editing a file.

Two things about `resolve_version` worth knowing before changing it.
`GITHUB_REF_NAME` is the **branch** name on a branch push, so reading it
unconditionally would stamp a build `feat/bevel-gear-generator`; `GITHUB_REF_TYPE`
is what separates the cases. And a tag that cannot be a version number is a hard
error rather than a fallback, because the quiet version of that failure is a
release labelled v0.2.0 containing an exe that says 0.1.0 — and the number is not
cosmetic, since it names the unpack cache directory. Nuitka takes up to four
dot-separated numbers and nothing else, so `v0.1.0-rc1` has nowhere to go.

### Continuous integration

[.github/workflows/ci.yml](.github/workflows/ci.yml) runs four jobs, three of
them on `windows-latest`:

```
test                 1001 collected (943 passed, 58 skipped in this environment) ~1 min
compiled-geometry    the probe above, --compare                   ~2 min cached
build                package.py, exe uploaded as an artifact      ~2 min cached
release              on a v* tag, attaches the exe to the release
```

**The runner is Windows and not for convenience.** pywin32 has no Linux wheel,
`tests/test_internal_geometry.py` imports `gears.sw.assembly_common` directly,
and the product runs nowhere else. On a Linux runner the suite would not fail so
much as quietly skip the parts that matter — which is also why the test job runs
with `-rs`. The suite is already CI-safe, `importorskip`-ing tkinter and
`gears.sw` and skipping explicitly when Tk cannot open a display; on this runner
none of those should trigger, and `-rs` is what makes it loud if one starts to.

Nuitka's C object cache is carried between runs, which is most of the wall clock
in the two compiling jobs.

**What CI cannot do is the whole of `sw/`.** There is no SOLIDWORKS on any
runner, so the 4,584 lines under `gears/sw/` are untouched by all of this, as is
the Build button inside the frozen exe. CI makes the division this README
already describes explicit rather than changing it: everything pure-Python is
gated automatically, and everything that touches a CAD seat is still Carl's by
hand.

---

## Layout

```
gears/
  involute.py    the planar involute core, shared by every type
  placement.py   clocking, speed ratio, transform packing
  preview.py     Scene/View, tooth and space boundaries, pan-zoom, drawing, DXF
  validate.py    Issue / ValidationResult primitives
  params_io.py   preset save/load mixin
  report_format.py  the shared three-column report layout
  gui.py         tkinter widgets and wiring, nothing else
  __main__.py    terminal report, dispatches on --type
  bevel/ spur/ planetary/ hypoid/
                 params.py geometry.py validate.py mesh.py preview.py report.py
  sw/            everything that touches pywin32 lives here
    session.py             COM connection, unit conversion, checked calls
    common.py              axis, blank, sections, loft cut, pattern, measure
    assembly_common.py     insert, place, find entities, mate, check
    bevel_part.py          bevel blank, dimension plan, spiral sections
    bevel_assembly.py      coincident apexes, axes at the shaft angle
    spur_part.py           spur and ring blanks, guide curve for helical teeth
    spur_assembly.py       parallel axes at a centre distance
    planetary_assembly.py  N + 2 components on concentric and orbiting axes
    hypoid_part.py         skew-axis hypoid blank and tooth loft
    hypoid_assembly.py     offset shafts, clocking and hypoid gear mate
tools/           standalone drivers and API probes, one per question asked
tests/           pure-Python; SOLIDWORKS is never involved
```

One sub-package per gear type; everything they share sits at the `gears/` level.
The split was made by moving code, not by rewriting it, so the comments in
`sw/common.py` are the bevel builder's hard-won ones and the measurements in
them were taken on the anchor bevel pinion. They are no less true of a spur gear.

**Three assembly builders, two part builders.** That is not an omission. A
planetary train is made of spur gears — an external sun, external planets and an
internal ring — so `spur_part` builds all three of its parts and only the
*assembly* has anything new in it. The same principle runs through
`gears/planetary/`: it owns the train, not the tooth.

The dependency direction is strict: `geometry` imports nothing but `params` and
`involute`, `preview` never imports tkinter, and nothing outside `sw/` imports
pywin32. That is what keeps the whole engine unit-testable without a CAD seat.

---

## The bevel geometry, in brief

Read the module docstring in [gears/bevel/geometry.py](gears/bevel/geometry.py)
before changing anything there — it is written to be read.

**Coordinate system.** Millimetres and radians. The gear axis is +Z and the
**pitch apex sits at the origin**, with the toothed body extending toward +Z.
Putting the apex at the origin is what makes the inner tooth section a plain
uniform scaling of the outer one. The SOLIDWORKS layer converts to metres at its
own boundary via `mm()`; nothing in `geometry` knows about that.

**Tooth form.** Tredgold's approximation: develop the back cone into a flat
"virtual" spur gear with `z / cos(delta)` teeth, generate a true involute there,
map it back onto the cone. Proportions follow the Gleason long-and-short-addendum
system, which is what GearTrax uses.

**The subtlest thing in the codebase.** The Gleason dedendum angle puts the root
cone apex exactly at the pitch apex, so root radii scale uniformly with cone
distance. The addendum angle is borrowed from the mate, which puts the face cone
apex *off* the origin — so tip radii do **not** scale uniformly and have to be
solved against the face cone (`tip_radius_at_cone_distance`). That asymmetry
catches everyone.

**The blank ends on the back cone**, not on a plane perpendicular to the axis.
Both members' back cones are perpendicular to the same pitch generator, so they
are tangent along it, which is what makes a meshed pair sit flush with no teeth
overhanging the mate. Truncating with a plane instead leaves the pinion's teeth
hanging 3.72 mm past the gear on the anchor set.

**The root rim.** The outer root point is where the tooth root emerges on the
back cone, so running the flat back straight through it leaves zero material
under the root at the heel — a wedge of included angle `90 - root_angle`. That is
a harmless 70° on a 17-tooth pinion but a 25° feather edge on its 43-tooth mate,
and it gets worse the further the ratio is from 1:1. `min_root_thickness` inserts
a short cylinder at the outer root radius before the flat back. Zero restores the
old outline exactly.

**Loft overshoot.** Both loft sections are pushed past the ends of the face
width by `end_overshoot`, because a cut finishing tangent to a real face of the
blank is rejected as zero-thickness geometry. The outer end is measured against
the *blank*, not the back cone — the root rim stands proud of the cone, and a
section that only cleared the cone would leave the rim uncut, bridging the tooth
spaces at the heel.


### The spiral, in brief

The tooth trace is a **circular arc laid in the generating crown gear's plane** —
what a face-milled Gleason cutter of radius `r_c` actually leaves — and each
member's trace is that one arc mapped onto its own pitch cone. It enters the
geometry as a single **phase** rotation per section: the section keeps its shape
and its cone distance and turns about the gear axis by however far the trace has
curved by that point along the face. Straight teeth are the *absence* of a
trace, so every phase is exactly zero and the same code path builds both.

```
rho^2      = Am^2 + r_c^2 - 2 Am r_c sin(psi_m)     places the arc
sin psi(A) = (A^2 + r_c^2 - rho^2) / (2 A r_c)      spiral angle at any A
theta_c(A) = acos((rho^2 + A^2 - r_c^2) / (2 rho A))  trace angle, crown plane
d_theta(A) = theta_c(A) / sin(delta)                 onto a cone
```

**One arc serves both members, and that is half the meshing condition.** Arc
length along the pitch circle at cone distance `A` comes to `A * theta_c` for
either member — every pitch-angle term cancels — so the two traces are the same
curve.

**The other half is which way they travel it, and getting that wrong cost a
build.** The two members roll on the crown gear in *opposite* senses — that is
what meshing is, and it is the same fact `angular_velocity_ratio` reports about
their axes — so the gear's phase is negated. This file used to claim the
opposite, that "no sign anywhere has to be chosen", and the test guarding it
compared the two arc lengths *without* their signs. An arc length is a
magnitude; two displacements of equal size can point opposite ways, and these
did. Measured on the anchor set, the two placed traces pulled apart to **11.53
mm**, agreeing only at the mean cone distance — which is where the phase is zero
by construction, so the one place every test looked was the one place that was
right. See `phase_at_cone_distance`, which carries the derivation.

Measured on the anchor spiral set (35 deg mean, cutter at Am):

```
face width      13.87 mm    the Gleason 0.30*Ao limit, against Ao/3 straight
spiral angle    30.074 / 35.000 / 40.599 deg at toe / mean / heel
sweep           38.790 deg on the pinion, 15.336 on the gear
face contact ratio  1.818
loft sections   11 and 12, against 2 apiece for a straight set
```

**The section count is measured, not solved, and a test is why.** The closed
form sized it over the face width while the loft runs over the face *plus* two
overshoots, and the trace turns fastest past the toe, so the interval down there
came out at 0.0293 mm against a 0.02 mm tolerance. Only intervals that reach the
blank are counted — a chord out in the overshoot bounds a cut that removes
nothing, and chasing it doubled the count for nothing.

The two members landing one apart is a coincidence of two effects nearly
cancelling, and worth knowing because the naive expectation is wrong: the pinion
sweeps 2.5 times as far, and the gear's tip stands 2.3 times further from the
axis it turns about, and the deviation is proportional to each.


### Zerol

**Zerol is a zero spiral angle with a real cutter radius.** The tooth is still
curved; it merely crosses the mean cone distance radially. That is why the
geometry switches on `is_curved` rather than on the spiral angle being zero, and
why `--zerol` is a flag of its own: `--spiral 0` alone is a straight gear, so
half-remembering the two-flag form gives you the wrong gear rather than an error.

```
.venv\Scripts\python.exe -m gears --module 2 --z1 17 --z2 43 --zerol
```

In the window it is the **Tooth trace** row — `straight` / `zerol` / `spiral` —
which is a combobox and not a "Zerol" tick for the reason `internal` is one: the
off state of that tick would have to mean straight *or* spiral, and there is no
honest label for that. The row is **derived**: it renders what the spiral angle
and cutter radius already come to, so typing 35 into the angle box moves it to
`spiral` on its own. Selecting a value writes those two rows rather than being
read back as a third input, which keeps one definition of what the tooth is —
`BevelSetParams.trace_kind`, shared with the terminal report.

Selecting `zerol` fills the cutter radius with the same Am that `--zerol` picks,
so the window and the command line hand back the same gear; press **Auto-size
blank** afterwards for the narrower face width a curved tooth takes. The cutter
row stays editable throughout — which matters, because the swing warning below
is advice you have to be able to act on.

Measured on the anchor Zerol set (0 deg mean, cutter at Am):

```
spiral angle    -11.265 / 0.000 / +9.394 deg at toe / mean / heel
sweep           0.898 deg on the pinion, 0.355 on the gear
face contact ratio  0.000
loft sections   12 and 20
```

**The face contact ratio is zero and that is the point of the gear**, not a gap
in the model. A Zerol tooth has no lengthwise overlap, which is what makes it a
straight-bevel substitute that throws no axial thrust at its bearings.

**It sweeps the least of the three and needs the most sections**, which is worth
stating because it looks like a contradiction. Sweep sizes the error of a chord
cutting the corner off a turn. A Zerol trace does not turn through an angle — it
*turns around*, at the mean cone distance — and a chord laid straight across that
bow misses it by the whole depth of the bow however small the endpoint angles
are. Sweep says nothing about that.

That is not a curiosity; it was a bug, and a quiet one. The count used to read
each loft interval's error off its two endpoint phases as `r (1 - cos(delta/2))`,
which is right only while the phase runs monotonically. On a Zerol set both ends
sit on the same side, the difference is 0.898 deg, and the measure was happy with
two sections — whose chord stands **1.3833 mm** off the trace against a 0.02 mm
tolerance. A Zerol pinion built that way is a straight tooth with a slight twist.
`_chord_deviation` now samples inside each interval and measures the chord
against the trace instead of inferring it from the ends.

**It is not only exactly-Zerol sets that turn.** The spiral angle crosses zero at
cone distance `sqrt(rho^2 - r_c^2)`, and with a nominal cutter that lands inside
the face for every mean angle below about 9.3 deg on the anchor set. So the
section count is not monotonic in spiral angle — it is U-shaped, and both ends of
the range cost more than the middle:

```
psi     0     5     9    10    15    25    35    44
count  12    12    11    10     9     9    11    15      anchor pinion, 0.02 mm
```

**A larger cutter flattens the swing**, on a Zerol set and a spiral one alike —
42.1 / 20.7 / 13.7 / 10.3 deg at 0.5 / 1 / 1.5 / 2 times Am on the Zerol set. The
limit is a straight tooth, since an infinite cutter sweeps a radial line, which
is why flattening the swing and keeping a spiral pull against each other and why
the nominal sits at Am. A default Zerol set therefore warns about its own 20.7
deg swing, and that warning is worth having rather than tuning away: real Gleason
Zerol practice does use a cutter larger than the cone distance.

**A Zerol tooth takes its bow from `hand`.** Nothing else can say which way it
bends — the mean spiral angle is zero for both hands, so the trace's sign is read
off the hand string rather than off the angle. That was inert until now: every
Zerol set bowed the same way whatever was asked for, because `-0.0 < 0.0` is
False. The pair meshes either way, as it does for a spiral set, because both
members take the same sign.

**Which sign of `hand` a catalogue would call right has not been measured.** The
arithmetic is symmetric, so nothing in the code can settle it — it needs someone
to build the anchor pinion and look down its axis. The pair meshes either way,
because both members take their hand from the same sign.

---

## The hypoid geometry, in brief

Hypoid sets use the Gleason/ISO Method 1 macro calculation. Unlike a bevel
pair, the two shaft axes do not intersect: `--offset` is the signed distance
between them along their common normal, positive on the assembly `+Y` side.
The input module is the wheel's outer transverse module, so the published
13/42 anchor is entered as `170/42`, with a 15 mm offset, 30 mm face width,
50 degree pinion spiral and 63.5 mm cutter.

The solver iterates the two pitch-cone angles and the hypoid offset angle at the
mean contact point. The anchor returns 21.288 / 68.324 degree pitch angles,
11.390 degree offset angle and 15.075 mm pitch-plane offset. The pinion and
wheel spiral angles differ: 50.000 / 38.609 degrees. The implementation uses
**ISO/Gleason Method 1 macro geometry with approximate Tredgold/back-cone tooth
surfaces**. The shared involute core supplies the two independent drive/coast
section flanks, while `hypoid.mesh` places the parts on skew axes rather than a
common apex. No true cutter-envelope or fully conjugate hypoid surface is
implemented.

The two lofts wind in opposite local phase senses. Their mean trace tangents
are resolved in the same skew-axis contact plane instead of copying the bevel
common-apex phase rule; the pinion and wheel tangents are not required to be
equal. Method 1 calculates distinct pinion and wheel spiral quantities and
physical tooth-face boundaries. The pinion's calculated member facewidth can
therefore differ from its physical pitch-cone boundary span, while the wheel's
input facewidth is split into its outer and inner spans. Separate terminal
sections may extend beyond those physical boundaries solely so a SOLIDWORKS
loft cut clears the blank. Assembly placement leaves the solved mean pitch
points coincident at zero backlash.

The circular cutter trace is only defined on its circle's radial domain. Phase
queries outside that domain are rejected for physical geometry. If a
construction-only terminal loft must go farther, its phase is extended by a
documented first-order tangent from the nearest physical tooth-face boundary;
the clamped inverse-trigonometric trace is never used there, and the physical
Method 1 face limits are unchanged.

The transverse Tredgold flank is a virtual external-spur involute. Below its
base circle the ordinary radial line is retained only while that root-side
boundary has a real clearance from the tooth-space centreline. If either
generated drive or coast flank reaches or crosses that line at a physical
inner, mean, or outer face station, the section is rejected with an explicit
`undercut/trochoid geometry not represented by the approximation` condition.
That condition means the virtual-spur approximation cannot supply the root; it
does not change the Method 1 macro geometry or prove that a cutter-generated
hypoid is impossible. A real root would require cutter and machine-setting
data that this repository does not calculate. Construction-only loft stations
reuse the nearest valid physical profile when necessary, so an unsupported
extension never creates a zero-width root slot or invalidates a valid physical
face.

Tooth thickness is balanced across the pair in the normal plane. The Method 1
thickness factor transfers thickness from the wheel to the pinion instead of
being added to both members; the two normal thicknesses sum to one normal pitch
minus the derived mean-normal backlash. Each is then converted by its own
spiral angle for the transverse Tredgold section. Consequently zero backlash
closes at the pitch point instead of leaving a spiral-angle-sized visible gap.

`min_root_thickness` is only the structural backing/root-rim dimension of the
blank. The approximate Tredgold tooth-space builder has a separate
`root_fillet_radius` input in millimetres. If it is omitted, the CAD-only
default is `0.1 * module`; the radius is fitted independently to both drive and
coast flanks. An explicitly supplied oversized value is rejected; the
module-based default warns and leaves an affected construction section sharp
when the independent flank geometry has no room for a circular blend. The
standalone SOLIDWORKS builder exposes the same value as
`--root-fillet-radius`. It is not the cutter-head
`cutter_radius`, nor a cutter blade-edge radius. A true hypoid root fillet
requires cutter blade and machine geometry that this implementation does not
model, so this circular transition must not be interpreted as a generated
cutter envelope.

The reported **spiral-bevel face overlap estimate** is not a calculated
operating hypoid contact ratio. ISO 23509:2016 Annex B.7.2 defines the
relationship below for spiral-bevel design selection. B.7.3 treats hypoids
separately when selecting the pinion spiral angle, so this project applies the
spiral-bevel relationship to the Method 1 wheel geometry only as an explicit
selection estimate:

```
q = b2 / Re2
K_z = q * (2 - q) / (2 * (1 - q))
epsilon_beta_est = Re2 / (pi * m_et2) * (
                     K_z * tan(beta_m2)
                     - K_z^3 / 3 * tan(beta_m2)^3
                   )
```

`m_et2` is the wheel outer transverse module (the input `module`), `b2` is
the wheel physical pitch-cone face span, `Re2` is its outer cone distance,
`beta_m2` is the wheel mean spiral angle. This is a face/overlap estimate,
distinct from a transverse profile contact ratio and from any true operating
hypoid contact ratio. The pinion calculated facewidth and pinion spiral angle
are not mixed into this wheel-side formula. The implementation takes the
magnitude only so hand reversal leaves the estimate unchanged. Because the
Tredgold loft is not a generated cutter envelope, the estimate must not be
read as a generated-flank contact ratio.

The Method 1 solver is deliberately bounded and reports non-convergence as a
validation error. This keeps an impossible offset from reaching the loft or the
SOLIDWORKS assembly builder.

## The spur geometry, in brief

Read the module docstring in [gears/spur/geometry.py](gears/spur/geometry.py).

**The involute is the same code.** Tredgold's approximation means the bevel
builder already develops the back cone into a flat virtual spur gear, generates
a true involute there, and bends it onto the cone. Strip the cone mapping and
the `k = Ai/Ao` scaling and what is left is a real spur gear, so the flat half —
`inv`, `top_land`, `max_tip_radius`, `flank_points`, `root_fillet` and the loop
assembly — lives in [gears/involute.py](gears/involute.py) and serves both.

**Coordinate system.** The axis is +Z and the **front face sits at z = 0**, with
the body running to `z = face_width`. A tooth space is centred on +X *at z = 0*,
so z = 0 is the clocking reference plane for the pair. Every other section is
that one rotated by the twist accumulated at its own height.

**Normal in, transverse out.** `module` and `pressure_angle` are the **normal**
values, because that is what a cutter is sold under. The involute lives in the
**transverse** plane, so the geometry works in `m_t = m_n / cos β` and
`tan α_t = tan α_n / cos β` throughout. Note the asymmetry this leaves and do
not tidy it away: the tooth depths are normal quantities (1.0 and 1.25 × m_n)
while the radii they are measured from are transverse ones.

**Straight teeth are never a special case.** They are `β = 0`, which makes the
twist zero and every section identical, and the same code path builds both.

**The two members are cut with opposite hands.** Not a convention — the gear is
placed on its parallel axis by a pure translation with no flip anywhere, so
same-hand helices would cross instead of meshing. What has to match between them
is the **axial pitch**, not the twist angle; the twists differ because the radii
do, and there is a test that says so.

**Undercut is reported, never designed around.** With no profile shift in the
parameter set there is nothing the geometry can do about it, and the model will
not even *show* it — the root below the base circle is drawn as a radial line,
not the trochoid a real cutter leaves — so a part that undercuts in reality
comes out of here looking sound. The warning says exactly that.

Two measurements worth keeping, both the opposite of the bevel case. A standard
spur tooth does **not** point on its own: 6 teeth at 25° still keeps 0.58 mm of
top land, where the Gleason long addendum nearly points a 17-tooth bevel pinion
unaided. And the transverse contact ratio only falls below 1.0 in the corner of
the allowed envelope — 6 teeth, 25°, a 40° helix, giving 0.894.


### Internal ring gears

`--internal` makes the second member a ring: teeth pointing inward, tip inside
the pitch circle and root outside it, on an annular blank.

**One identity carries the whole thing.** An internal gear's tooth *space* has
the shape of an external gear's *tooth* — both narrow as the radius grows,
following `psi0 - inv(alpha_r)`, where an external gear's space widens instead.
That sign is why `internal_flank_points` is a second function rather than
`flank_points` called cleverly: no substitution of `psi0` or `half_pitch` turns
one form into the other. Building `psi0` from the **space width** rather than
the tooth thickness is the entire substitution.

Three things flip together:

* the **centre distance** is the difference of the tooth counts, not the sum
* the **helix hands agree** rather than oppose — no flip in the placement either
  way, but a ring wraps around its pinion instead of facing it
* the pair **turns the same way**, which is the sign a planetary train is made of

**The ring is placed at -a, and that is forced.** Internally tangent pitch
circles touch on the far side of the small one, so at `-a` the contact lands at
the pinion's angle 0 where its tooth space already is. At `+a` it lands at pi,
and the pinion would need clocking too — and the pinion is the one member in
this codebase that never does. The ring's own clocking is then half an angular
pitch, with no parity case.

Two measurements worth keeping. **The contact ratio's two signs flip together,
and the dangerous failure is forgetting both**:

```
both flipped (correct)             g =  11.431    ratio 1.936
neither flipped                    g =   9.913    ratio 1.679
only the branch flipped            g = -17.298    ratio 0
only the centre distance flipped   g =  38.643    ratio 6.545
```

Half-flipping is loud. Forgetting the flip entirely is quiet — 1.679 against a
true 1.936 is an ordinary-looking number that passes every validator — which is
why the test for it solves the length of action longhand.

And **a ring needs a minimum tooth count of its own**, separate from the
difference rule, before its tip clears its own base circle:

```
14.5 deg    63 teeth
20   deg    34 teeth
25   deg    22 teeth
```

A *lower* pressure angle needs *more* teeth, which is backwards from instinct
and is why the anchor ring has 60. Trimming interference is guarded by the
ten-tooth difference rule and is **not** computed: the first attempt shipped a
formula that flagged the anchor pair by comparing the two tip circles directly,
when a meshing internal pair's tip circles are supposed to overlap — that is
where the mesh is.

---

## The planetary train, in brief

Read the module docstrings in [gears/planetary/mesh.py](gears/planetary/mesh.py)
and [gears/planetary/geometry.py](gears/planetary/geometry.py).

A sun, N planets and an internal ring, composed out of the spur type rather than
owning a second involute. The three members come from calling
`spur.geometry.compute_set` twice — an external pair for sun-and-planet, an
internal one for planet-and-ring — so this package owns the **train**, not the
tooth.

**The ring's tooth count is not an input.** Both centre distances are the same
distance measured twice, which forces `z_ring = z_sun + 2 z_planet` exactly.

**Every clocking comes out of one relation**, and it is not a new rule:

```
external   r_A (phi_A - c_A) + r_B (phi_B - c_B) = p/2 + n p
internal   r_A (phi_A - c_A) - r_B (phi_B - c_B) = p/2 + n p
```

where `phi_X` is the contact direction seen from X's own centre. Substituting a
plain spur pair gives back `placement.gear_clocking`; substituting an internal
pair gives back `spur.mesh.internal_gear_clocking`. Both are tests, so the
train's arithmetic is anchored to two answers that were already right. What
falls out is

```
c_planet(k) = phi_k (z_s + z_p) / z_p + gear_clocking(z_p)
c_ring(k)   = phi_k (z_r + z_s) / z_r + gear_clocking(z_p) z_p / z_r + pi / z_r
```

**The assembly condition then arrives as a consequence rather than a rule from a
table.** The ring is one part and can only have one clocking, so `c_ring(k)` has
to come out the same for every planet modulo its angular pitch — and that
reduces to `(z_sun + z_ring)` being divisible by the planet count. A set that
fails it does not have a planet slightly out of place; it has no consistent ring
clocking at all. Measured on a 27/17/61 set with three planets: planet 0 fits
perfectly — it is the one the ring was clocked against — and planets 1 and 2
each drive 855 samples into the ring.

**The carrier is held, not modelled.** The assembly is the carrier-stationary
configuration: every member spins about an axis fixed in space, which is a real
planetary arrangement — it is the one giving the ring-to-sun ratio of
`-z_r/z_s` — and needs no carrier solid to be correct. A carrier part with
orbiting planets is a genuinely different assembly, because the planets' axes
move and so cannot be mated to the assembly's own planes at all.

On the anchor set:

```
ring = sun + 2 planet         60 = 24 + 36
assembly  (z_s + z_r) % N     84 / 3 = 28 exactly
centre distance, both meshes  42.0000 mm, equal
planet tip diameter           40.000 mm vs 72.746 mm neighbour spacing
ring held:   sun / carrier    3.5000
carrier held: ring / sun     -2.5000
```

---

## Backlash

For spur, bevel and internal gears, `--backlash 0.1` is a circular backlash in
millimetres at the applicable pitch circle. For hypoids it has a deliberately
different, explicit convention: `--backlash 0.2` is the ISO outer transverse
backlash `j_et2`, measured at the wheel outer pitch cone. The hypoid solver
converts that one pair-level input to mean transverse and mean normal backlash
using the two members' own spiral angles.

**Taken off the tooth, not added to the centre distance.** Both are real ways to
build backlash into a pair and only one of them leaves the rest of the model
alone: thinning the tooth keeps the centre distance, the mounting distances and
the cone angles at their nominal values, so every other number in the report
still means what it said.

**Split evenly, so the mesh sees it once.** Each member loses `backlash / 2` of
arc thickness. The failure worth guarding against is applying the whole figure to
each member — which doubles the play while every single-member check still
passes — so the test states it as a property of the *pair*: the two tooth
thicknesses at the pitch circles plus the backlash come to exactly one circular
pitch. An internal pair reaches the same place from the other side, because a
ring gear's `psi0` is built from its **space** width: the backlash widens the
ring's space rather than thinning its tooth directly, and the pair sum is what
says the two descriptions agree.

**On a bevel set it is quoted at the outer end**, like the module it is measured
against. Every inner section is a uniform scaling of the outer one by
`k = A/Ao`, and the backlash is a length on that section like any other — so a
tooth thinned by 0.05 mm at the heel is thinned by 0.033 mm at the toe on the
anchor set. That is what a cutter leaves, not an approximation of one, and there
is a test that differences two backlashes to say so exactly.

**Zero is not just the default, it is a condition other code relies on.** With no
backlash the flanks touch exactly at the pitch point, which is why
`check_interference` runs with `TreatCoincidenceAsInterference` off and why the
verified helical pair came back with "one zero-volume tangency where the flanks
touch". A set built with backlash should show that tangency gone.

The validators say two things about it: a negative value is an error in all
three, and anything past 5 % of the circular pitch is a warning — a rail against
a typo, not a design standard, since a real backlash on a 2 mm module is well
under 1 %. What backlash actually *breaks* is left to the checks that can see the
tooth: a millimetre of it on a 6-tooth, 25° spur pair thins the tooth away
entirely, and that comes back as a pointed-tooth error naming the member.

---

## The SOLIDWORKS build

`build_gear` in [gears/sw/bevel_part.py](gears/sw/bevel_part.py) runs five steps:

1. reference axis along Z (intersection of the Top and Right planes)
2. blank: meridian outline sketched on the Top Plane, fully dimensioned with
   *driving* dimensions, revolved 360°
3. 3D sketches of the tooth-space section — two for a straight gear, and as
   many as the spiral trace needs for a curved one (11 and 12 on the anchor
   spiral set, 12 and 20 on the Zerol one)
4. loft cut through them, with a guide curve when the teeth are curved
5. circular pattern of that cut, `z` instances about the axis

The blank does not change between straight and spiral. A spiral changes where
along the face each section sits about the axis, not the meridian outline it is
cut out of, so the whole dimension plan below is untouched by it.

Every blank dimension then gets a named global variable and is driven from it,
so the part stays editable as a parametric model rather than dead geometry:

```
"CrownRadius" = 19.922mm
"CrownRadius@Sketch1" = "CrownRadius"
```

`BackFaceToApex` is the one derived dimension — `"OuterRootToApex" +
"MinRootThickness"` — so editing the rim grows the blank backward instead of
eating into the teeth.

### The assembly

`build_set` in [gears/sw/bevel_assembly.py](gears/sw/bevel_assembly.py) inserts both
parts, writes their placement transforms, then floats them and mates them. Each
member ends up with five degrees of freedom removed and keeps the sixth — the
spin about its own axis:

```
apex        component origin coincident with the assembly origin      (3)
axis        component axis coincident with the assembly Top plane     (1)
direction   pinion: axis also coincident with the Right plane         (1)
            gear:   angle mate to the pinion axis = the shaft angle   (1)
```

A **gear mate** between the two reference axes then ties those two remaining
freedoms together in the ratio z1:z2. The assembly is deliberately left under
defined by one degree of freedom — fully defined would mean the teeth could not
turn — and each component's `GetConstrainedStatus` is read back to confirm it.

Every mate is added with `swMateAlignCLOSEST`. That is what lets the two
mechanisms cooperate: the components are already sitting exactly where they
belong, so "closest" resolves each mate's alignment against the arrangement in
front of it and nothing moves. ALIGNED or ANTI\_ALIGNED is a coin toss that
flips a part half the time. No mate touches the spin, so the clocking written by
the transform survives — the gear mate couples the two spins wherever it finds
them rather than choosing a phase.

`mesh.angular_velocity_ratio` shows the two members must turn in **opposite**
senses about their outward axes, and that the ratio is exactly z2/z1 at *any*
shaft angle — every shaft-angle term cancels, which is not obvious until you
substitute the pitch cone relation. What that does not settle is how SOLIDWORKS
reads a gear mate's Reverse flag against two reference axes, which the API will
not report. Measured by hand on the anchor set: with the pinion axis selected
first and Reverse off, the pair turns the right way. `--reverse-gear` flips it
if that ever stops holding.

It had to be measured by hand because **a gear mate is applied by the
interactive drag solver and by nothing else**. Writing a component transform and
rebuilding, adding an angle mate that drives the pinion's last freedom, and
`IDragOperator` were each measured to turn the pinion exactly as asked and leave
the gear at 0.0000° — see [tools/probe_gear_sense.py](tools/probe_gear_sense.py),
which is kept for its three dead ends. The same fact is why the transform-written
clocking is safe: the gear mate never gets the chance to shift it.

`--no-mates` places the pair and leaves it floating, which is the older
behaviour and worth having when a mate is the thing under suspicion.

### The spur build

`build_spur` in [gears/sw/spur_part.py](gears/sw/spur_part.py) runs the same
five steps on the shared helpers in [gears/sw/common.py](gears/sw/common.py).
Three things differ.

The blank is a stepped cylinder, so every dimension is linear and none is
angular — this builder needs neither `_angle_position` nor the quadrant argument
it exists to get right. Its global variables are `BoreRadius`, `TipRadius`,
`FaceWidth`, `HubRadius` and `HubThickness`, with `HubBackFace` derived as
`"FaceWidth" + "HubThickness"` so editing the hub grows it backward instead of
walking the gear's own back face into the teeth.

**A ring gear reuses this builder unchanged.** Its blank turns out to be the
same four-line meridian profile a hubless external blank has — a rectangle — so
the whole degree-of-freedom count and every relation holds. What differs is only
what the two radii *are*, and so what they are called: the inner one is the
ring's tip circle rather than a bore, and the outer is `RimRadius` rather than
`TipRadius`. Naming them the external way would leave someone editing
"BoreRadius" to move a tooth.

**The front face is held by a relation, not a dimension.** Its dimension would
measure zero and a zero-length dimension cannot be created. That is not a
workaround: "the front face sits at z = 0" is a statement about the coordinate
system, not a number anyone would edit.

**Helical teeth need more sections, not just a guide curve.** Two identical
sections loft to a prism, which is the tooth. Two *rotated* sections loft to a
surface that meets the helicoid only at the ends and falls inside it in between,
because a loft carries each profile point along a straight chord and a helix is
an arc.

A guide curve does not fix that, and finding out cost a build. Measured on the
anchor helical pinion, cut from two sections plus a guide:

```
                        two sections + guide   six sections + guide
mid-face vs end profile        0.198 mm               0.011 mm
volume vs true helicoid        +0.900 %              -0.068 %
max radius error               +0.83 %               +0.08 %
root radius read back      15.13 - 15.21 mm      15.094 - 15.105 mm
```

The guide pins the one point it runs through — the tip agreed exactly both times
— and leaves the rest of the profile interpolated. So `section_heights` solves
the chord's sagitta, `r * (1 - cos(delta / 2))` at the tip radius, against
`MAX_SECTION_SAGITTA_MM = 0.02`, and the loft runs through as many sections as
that asks for. Straight teeth still take two, because their twist is zero.

The guide stays as well: it costs one sketch and it pins the cap exactly. It is
a sampled 3D spline rather than an `InsertHelix` feature — no agreement with
SOLIDWORKS about pitch, start angle or hand, which is three chances to be off by
a sign on a part where a sign error is a gear that will not mesh. It runs through
the cap's centreline vertex, and `split_cap` puts a real vertex there, because a
guide that only passes *near* a spline is the classic way a guided loft fails.

`build_spur_set` in [gears/sw/spur_assembly.py](gears/sw/spur_assembly.py)
places the pair the same way — transforms first, mates after — but on parallel
axes a centre distance apart:

```
pinion      origin coincident with the assembly origin                (3)
            axis coincident with the assembly Top plane               (1)
            axis coincident with the assembly Right plane             (1)

gear        axis parallel to the pinion axis                          (2)
            axis coincident with the assembly Top plane               (1)
            origin coincident with the assembly Front plane           (1)
            axis at distance a from the pinion axis                   (1)
```

The gear's half is not the pinion's, and the obvious arrangement is wrong three
ways. **Its axis runs along +Z, so it cannot lie in the Front plane** — that is
the XY plane, and only Top and Right contain a +Z direction, with Right forcing
the gear onto x = 0, which is exactly where it must not be. **Nothing else
locates it axially**: a bevel pair takes all three translations from putting both
origins on the assembly origin, but here they are `a` apart, so the gear's origin
— which sits at its front face — is mated to the Front plane to put both fronts
on z = 0. And the direction is a **parallel** mate, because an angle mate at zero
degrees solves to 180 as readily as to 0.

The centre distance is a distance mate between the two axes. This is also the
first caller of `to_array_data`'s translation argument; a bevel pair always
passed zero.

### What the probe measured

`MARK_LOFT_GUIDE = 2` is the one constant in `session.py` not read out of
`swconst.tlb` — guide curves have no enum, so 2 was only what the API reference
gave. [tools/probe_helix_loft.py](tools/probe_helix_loft.py) settled it and three
other questions the documentation will not, by measuring the volume a cut removes
against an unguided control — a mark SOLIDWORKS ignores hands the control's
number straight back:

```
no guide      372.969 mm3     the control
mark 2        299.458 mm3     used as a guide curve      <- MARK_LOFT_GUIDE
mark 4        331.042 mm3     used, but as the centreline
mark 8        372.969 mm3     ignored
mark 16       372.969 mm3     ignored
```

A sampled 3D spline is accepted as a guide. A guided cut still patterns with
`GeometryPattern` on. And the guide may overrun the profiles, stop exactly on
them, or stop short — all three were accepted.

**The gear mate's Reverse sense is still not measured for a spur pair**, and the
bevel answer does not transfer: those axes stand at 90° to each other and these
are parallel. It cannot be settled from the API, because a gear mate is applied
by the interactive drag solver and by nothing else. `--reverse-gear` is there
from the first build. Drag the pinion, watch which way the gear goes, and record
the answer in the module docstring of `gears/sw/spur_assembly.py`.

The **internal** pair is a separate measurement again, and so is each of the
planetary train's two meshes. Nothing transfers between them: an external pair
turns in opposite senses and an internal one in the same sense, which is a fact
about the geometry that says nothing about how SOLIDWORKS reads a flag.

**The planetary train's two senses have now been measured, and they differ.**
Dragging the sun on the anchor set: sun:planet turns right with Reverse off, and
planet:ring turned the ring backwards until Reverse went *on*. So the two meshes
carry different flags — `SUN_PLANET_GEAR_MATE_FLIP` and
`PLANET_RING_GEAR_MATE_FLIP` in
[gears/sw/planetary_assembly.py](gears/sw/planetary_assembly.py) — and
`--reverse-gear` flips both *away* from those, which makes it an escape hatch
rather than a setting with a right value. It is the first confirmation that a
mesh's kind reaches the Reverse flag at all, and it still does not transfer: the
plain internal spur pair is its own measurement.

### Hard-won facts about this API

These were each found by breaking something. Don't undo them:

* **Late binding only.** `EnsureDispatch` cannot introspect `ISldWorks`. No
  argument coercion is done for you: point arrays need an explicit
  `VT_ARRAY | VT_R8`, null object parameters need a `VT_DISPATCH` holding None,
  and an `int` where a `double` is expected raises "Type mismatch".
* **Failures are silent.** The API returns `None` or `False` rather than
  raising, so every call goes through `require()`.
* **Some members resolve as properties.** Late binding decides on *first*
  access and caches it, so `flag_methods` has to be called at creation time —
  by the time the first read fails it is already too late.
* **Transforms are column-major.** Reading `IMathTransform.ArrayData` row-major
  silently mirrors the part.
* **`GeometryPattern` must be on.** Off, SOLIDWORKS re-solves the loft cut at
  every rotated position, and a cut driven by two 3D sketch profiles will not
  re-solve that way — the pattern is rejected outright.
* **The loft cut splits the body rather than removing material.** It leaves the
  tooth space behind as a second solid, which also blocks the pattern;
  `_drop_offcut` discards it.
* **Angular precision has to be raised in the document.** An angular equation is
  re-parsed at the document's precision on every rebuild, and at the default two
  decimals a driving cone angle moves the geometry.
* **The assembly is *placed* by writing transforms, and constrained only
  afterwards.** A bevel pair needs coincident apexes, axes at the shaft angle,
  *and* clocked teeth — three conditions the mate solver can satisfy in the
  wrong way. Placing by transform settles all three by arithmetic; the mates
  then go onto a pair that is already right.
* **Components added through `AddComponent5` arrive fixed.** Fixed outranks
  every mate on them, so a gear mate can be added, look correct in the tree,
  and do nothing at all when the assembly is dragged. `UnfixComponent` first.
* **`AddMate5` reports its refusal byref.** The return value can be a mate
  object that is not in the assembly; the `ErrorStatus` argument is the one
  that tells the truth, and `swAddMateError_NoError` is 1, not 0.
* **An origin cannot be mated as a feature.** Selecting the OriginProfileFeature
  is what a user does in the tree, but over the API it lands in the selection
  list as `swSelSKETCHES` and `AddMate5` refuses it — returning None with error
  status 0, "unknown error". Reach through to the single sketch point inside it,
  which reports `swSelEXTSKETCHPOINTS` and mates first time.
* **Marks do not matter to `AddMate5`.** It mates whatever is in the selection
  list, which is what lets `ISketchPoint.Select4` — which has no mark parameter
  at all — be used for one entity and `IFeature.Select2` for the other.
* **An angle mate between a line and a plane is measured *to the plane*.** So it
  cannot carry a position round an axis: every planet axis runs along +Z and the
  Top plane contains +Z, which makes that angle identically 0 at every station.
  Asking for 120° came back "unknown error" — a refusal to build an
  unsatisfiable mate. The station is a **distance**, `a sin(theta)`.
* **A distance mate's sign lives in the `Flip` flag, not in the alignment.**
  swMateAlignCLOSEST does *not* keep a component on the side the transform
  already put it, which is the one place the place-then-constrain division of
  labour does not hold on its own. Mating two planets to the same |y| landed
  both on the +y station, one inside the other — 18552.6958 mm3 of overlap,
  and every planet still exactly 42 mm from the axis, so a radius check saw
  nothing. Pass `flip=offset < 0`.
* **Gear mates are budgeted against the assembly's freedoms.** Each removes one,
  so N + 2 members admit exactly N + 1 before the train is fully defined and the
  next one is refused with "the mate would over-define the assembly". Two per
  planet is one too many from the third mate on; the mates have to form a
  **spanning tree** over the members.

`tools/smoke_com.py`, `tools/probe_dimension.py`, `tools/probe_mate.py`,
`tools/probe_gear_sense.py`, `tools/probe_helix_loft.py` and
`tools/probe_hypoid_loft.py` exist to answer API questions in isolation before
trusting an answer inside a build. Add to them rather than debugging inside a
builder.

---

## Testing

The current run collects 1001 tests (943 passed and 58 skipped), all pure Python
and all fast. They are closed-form checks on the
geometry — cone distances agreeing between members, tooth tips landing on the
mate's back cone, centre distance solved two independent ways, loft sections
clearing the blank, the guide curve touching a vertex that exists in every
profile, the gear ratio solved back out of the rolling condition — plus the
validators, the preview arithmetic, the CLI dispatch and the GUI's parameter
plumbing.

The rule that makes them worth having: **express each COM precondition as a pure
Python property of the geometry.** `test_the_guide_helix_touches_the_cap_vertex_of_both_end_sections`
exists because a guided loft fails when its guide only passes *near* a spline,
and that is cheaper to assert here than to discover in SOLIDWORKS.

Three tests earn their keep above the rest, and all three ask the same kind of
question — *does this actually mesh?* — as something a computer can answer:

* `test_both_members_traces_coincide_along_the_common_pitch_generator` — the
  spiral bevel meshing condition, asserted as the **distance in space** between
  the two placed traces. It used to compare two arc lengths and pass on a pair
  that interfered in 17 regions; a length cannot tell you which way it points
* `test_the_placed_pair_interlocks_without_the_two_bodies_overlapping` — samples
  the pinion's metal and asks whether any of it is inside the ring's
* `test_the_whole_train_interlocks_without_any_two_bodies_overlapping` — the
  same for all N + 2 members of a planetary set at once

**Each of them is paired with a test that makes it fail on purpose**, because a
check that cannot fail proves nothing. Mis-clocking by half an angular pitch —
exactly the difference between aiming a tooth centre and a tooth space at the
contact — takes the internal pair from 0 overlapping samples to 2232 and the
planetary train from 0 to 1890.

`sw/` is not covered, because it needs a CAD seat. Anything in there is verified
by building the anchor set in SOLIDWORKS and looking at it. **Carl does that
verification, and his hands-on result outranks any API probe.** If you change
`sw/`, say plainly that you have not run it.

The Method 1 hypoid anchor was built live on SOLIDWORKS 2026 using the
approximate Tredgold/back-cone tooth surfaces described above. Both parts are
single solids (13-tooth pinion: 59 faces; 42-tooth gear: 175 faces), the saved
assembly measures 90.0000 degrees between its shafts and 15.0000 mm axis offset
with zero reported placement error, and its 13:42 gear mate leaves both members
under defined so the pair articulates. The skew contact solution clocks the
pinion and wheel traces in opposite local senses, resolves their mean tangents
in the shared contact plane, and keeps the solved mean pitch points coincident
at zero backlash without an assembly-clearance offset.

What the spur builders have been run against, on SOLIDWORKS 2026 SP3:

```
straight pinion   volume within 0.041 % of the closed form, bore exactly 4.25 mm
helical pinion    tip radius 19.5997 mm at three heights, twist linear and
                  symmetric about mid-face, volume within 0.068 %
straight pair     centre distance exact to 6 decimals, axes 0.000000 deg apart,
                  no interference, both members under defined - it articulates
helical pair      the same, with one zero-volume tangency where the flanks touch
```

Everything above was then re-run through the **window's own Build button**,
along with the three builders that had never been run at all. Driving it from
the GUI rather than from `tools/` is what found the four bugs below: three of
them are on the path from the button to the builder, and no `tools/` driver
crosses it.

```
internal pair    centre distance 42.0000 mm exact, ring clocked 3.0000 deg -
                 half its angular pitch, as predicted - no interference,
                 it articulates
planetary train  3 planets at (42.0000, 0), (-21.0000, +-36.3731), worst
                 station error 0.00e+00 mm, ring clocked 6.0000 deg, no
                 interference, 21 mates, it articulates
backlash         0.1 mm on the helical pair: the zero-volume tangency is gone
                 and interference reads none, which is the observable this
                 section asked for
spiral bevel     0.0631 mm3 in 5 regions, shaft angle exact - see below
zerol bevel      0.0343 mm3 in 4 regions, mates and articulates
```

**The spiral bevel pair's interference is fixed, and the cause was a sign.** It
stood at 17 regions and 103.5074 mm3 — one region per pinion tooth, so
systematic rather than a stray sliver — and two things were measured and ruled
out at the time:

```
loft discretisation   11 sections -> 21 (max_sagitta 0.02 -> 0.005 mm)
                      moves it 103.5074 -> 103.5058 mm3.  Not the chord error.
tooth thickness       0.3 mm of backlash - three times what cleared the
                      helical pair outright - moves it to 91.0275 mm3.
                      Not a thin margin either.
```

Both were right, and both pointed away from the answer. The fault was in
`bevel/geometry.py` after all: **the gear's tooth trace ran the wrong way round
its axis.** Both members took the same sign off the one crown arc, when meshing
requires them to roll on that crown in opposite senses. The section above has
the derivation; what matters here is that the pure-Python meshing test *passed*
on the broken pair, because it compared two arc lengths and an arc length has no
direction in it. Rebuilt, on SOLIDWORKS 2026 SP3:

```
                      before            after
spiral 35 deg    103.5071 mm3 / 17    0.0631 mm3 / 5
zerol             27.4611 mm3 /  6    0.0343 mm3 / 4
straight               none            none        unchanged
```

0.06 mm3 across 5 regions is tangency-level — the same character as the helical
spur pair's "one zero-volume tangency where the flanks touch" — against a tooth
volume three orders of magnitude larger. The straight pair is untouched, since
it has no trace to get the sign of.

The meshing test now places both traces and measures the distance between them,
and is paired with one that removes the negation and requires the old 11.53 mm
divergence back. That pairing is the point: the test that let this through could
not fail.

Three things are still waiting on a hand on the drag solver, and none of them
inherits from any of the others:

```
spur pair, external   which way it turns             open since the last pass
spur pair, internal   the opposite sense - separate measurement
spiral bevel          whether `hand` matches what a catalogue calls right

planetary             settled: sun:planet Reverse off, planet:ring Reverse on
```

### The MCP as an inspection surface

There is a SOLIDWORKS MCP server configured in `.mcp.json`, which is gitignored
rather than shared — it names the path to a checkout of the server elsewhere on
one disk, so it has to be written per machine. It does not build
these gears — it has no mate or component-insert operations at all, so an
assembly is beyond it — but it is a good way to *look at* what the builders
produced without leaving the terminal: `list_bodies` and `body_volume` against
the closed-form blank volume, `probe_section` and `section_screenshot` on a
transverse plane compared with the Python profile at the same z, `check_clearance`
between components, `take_screenshot` for the visual — which on a planetary
assembly is the quickest way to see whether every planet found its station.

That is evidence, not proof, and it ranks below Carl looking at the part.

---

## House style

The comments in this codebase carry *why*, not *what*, and they are unusually
long where the reason is unusually subtle. Several record measurements —
"measured on the anchor pinion, setting CrownRadius to 22 mm walks the crown
from (19.9220, 22.5585) to (22.0000, 21.0000)". Keep that: a number someone
measured once is worth more than a paragraph of reasoning about what should
happen. Match the surrounding density when you add code.
