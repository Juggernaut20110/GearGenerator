# BevelClaude

Generates straight bevel gear pairs as real SOLIDWORKS parts and a meshed
assembly, from nine numbers. Pure-Python geometry engine, a tkinter front end,
and a COM driver that builds the solids.

Given a module, tooth counts, pressure angle, shaft angle, face width, bore, hub
and root rim, it produces:

* a fully dimensioned blank sketch, revolved, with every dimension driven by a
  named global variable you can edit in SOLIDWORKS afterwards
* true involute teeth, lofted between two 3D-sketch sections and circular
  patterned
* both members plus an assembly with the pair correctly clocked and meshing,
  mated so it articulates: a gear mate couples the two, and dragging either
  member turns the other in the right ratio

The anchor case used throughout the code and tests is **m=2, 17×43 teeth, 20°
pressure angle, 90° shafts**.

---

## Running it

The interpreter is the venv one. Always. There is no global install.

```
.venv\Scripts\python.exe run.py                      # the GUI
.venv\Scripts\python.exe -m bevelgear --module 2 --z1 17 --z2 43
.venv\Scripts\python.exe -m pytest -q                # 299 tests, no SOLIDWORKS
```

Building actual geometry needs SOLIDWORKS running (or installed — the session
will start it):

```
.venv\Scripts\python.exe tools\build_gear.py --member gear
.venv\Scripts\python.exe tools\build_set.py --z1 17 --z2 43
```

`--help` on any of those lists the parameter flags. Output lands in `out/`,
which is gitignored along with all SOLIDWORKS file types.

---

## Layout

```
bevelgear/
  params.py      the nine inputs; JSON save/load for GUI presets
  geometry.py    all the maths - the heart of the project
  validate.py    errors block a build, warnings don't
  mesh.py        how the two members sit relative to each other
  preview.py     scenes, pan/zoom arithmetic, DXF and CSV export
  gui.py         tkinter widgets and wiring, nothing else
  __main__.py    terminal report, no SOLIDWORKS involved
  sw/            everything that touches pywin32 lives here
    session.py   COM connection, unit conversion, checked-call discipline
    part.py      builds one gear: blank, sections, loft cut, pattern
    assembly.py  builds both, places them, mates them into a turning set
tools/           standalone drivers and API probes, one per question asked
tests/           pure-Python; SOLIDWORKS is never involved
```

The dependency direction is strict: `geometry` imports nothing but `params`,
`preview` never imports tkinter, and nothing outside `sw/` imports pywin32.
That is what keeps the whole engine unit-testable without a CAD seat.

---

## The geometry, in brief

Read the module docstring in [bevelgear/geometry.py](bevelgear/geometry.py)
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

---

## The SOLIDWORKS build

`build_gear` in [bevelgear/sw/part.py](bevelgear/sw/part.py) runs five steps:

1. reference axis along Z (intersection of the Top and Right planes)
2. blank: meridian outline sketched on the Top Plane, fully dimensioned with
   *driving* dimensions, revolved 360°
3. two 3D sketches — the tooth-space section at each end of the face width
4. loft cut between them
5. circular pattern of that cut, `z` instances about the axis

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

`build_set` in [bevelgear/sw/assembly.py](bevelgear/sw/assembly.py) inserts both
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

`tools/smoke_com.py`, `tools/probe_dimension.py`, `tools/probe_mate.py` and
`tools/probe_gear_sense.py` exist to answer API questions in isolation before
trusting an answer inside a build. Add to them rather than debugging inside
`build_gear`.

---

## Testing

299 tests, all pure Python, all fast. They are closed-form checks on the
geometry — cone distances agreeing between members, tooth tips landing on the
mate's back cone, loft sections clearing the blank, the blank outline's shape,
the gear ratio solved back out of the rolling condition — plus the validator,
the preview arithmetic and the GUI's parameter plumbing.

`sw/` is not covered, because it needs a CAD seat. Anything in there is verified
by building the anchor set in SOLIDWORKS and looking at it. **Carl does that
verification, and his hands-on result outranks any API probe.** If you change
`sw/`, say plainly that you have not run it.

---

## House style

The comments in this codebase carry *why*, not *what*, and they are unusually
long where the reason is unusually subtle. Several record measurements —
"measured on the anchor pinion, setting CrownRadius to 22 mm walks the crown
from (19.9220, 22.5585) to (22.0000, 21.0000)". Keep that: a number someone
measured once is worth more than a paragraph of reasoning about what should
happen. Match the surrounding density when you add code.
