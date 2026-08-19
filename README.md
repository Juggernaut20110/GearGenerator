# BevelClaude

Generates gear pairs as real SOLIDWORKS parts and a meshed assembly, from a
handful of numbers. Pure-Python geometry engine, a tkinter front end, and a COM
driver that builds the solids.

Two gear types, sharing everything they can:

* **straight bevel** — module, tooth counts, pressure angle, shaft angle, face
  width, bore, hub and root rim
* **involute spur**, straight or helical — normal module, tooth counts, normal
  pressure angle, helix angle and hand, face width, bore and hub

Either one produces:

* a fully dimensioned blank sketch, revolved, with every dimension driven by a
  named global variable you can edit in SOLIDWORKS afterwards
* true involute teeth, lofted between two 3D-sketch sections and circular
  patterned
* both members plus an assembly with the pair correctly clocked and meshing,
  mated so it articulates: a gear mate couples the two, and dragging either
  member turns the other in the right ratio

The anchor case used throughout the code and tests is **m=2, 17×43 teeth, 20°
pressure angle** — with **90° shafts** for the bevel set, and a **15° helix**
for the helical spur set, so the numbers sit beside each other.

---

## Running it

The interpreter is the venv one. Always. There is no global install.

```
.venv\Scripts\python.exe run.py                      # the GUI, both types
.venv\Scripts\python.exe -m gears --module 2 --z1 17 --z2 43
.venv\Scripts\python.exe -m gears --type spur --module 2 --z1 17 --z2 43 --beta 15
.venv\Scripts\python.exe -m pytest -q                # 653 tests, no SOLIDWORKS
```

`--type` defaults to `bevel`, which is what the tool generated before there was
a choice. A flag belonging to the other type is refused rather than ignored, so
`--sigma` on a spur set is an error and not a silent no-op.

Building actual geometry needs SOLIDWORKS running (or installed — the session
will start it):

```
.venv\Scripts\python.exe tools\build_gear.py --member gear
.venv\Scripts\python.exe tools\build_set.py --z1 17 --z2 43
.venv\Scripts\python.exe tools\build_spur.py --member gear --beta 15
.venv\Scripts\python.exe tools\build_spur_set.py --z1 17 --z2 43
```

`--help` on any of those lists the parameter flags. Output lands in `out/`,
which is gitignored along with all SOLIDWORKS file types.

---

## Layout

```
gears/
  involute.py    the planar involute core, shared by both types
  placement.py   clocking, speed ratio, transform packing
  preview.py     Scene/View, pan-zoom arithmetic, drawing, DXF
  validate.py    Issue / ValidationResult primitives
  params_io.py   preset save/load mixin
  report_format.py  the shared three-column report layout
  gui.py         tkinter widgets and wiring, nothing else
  __main__.py    terminal report, dispatches on --type
  bevel/ spur/   params.py geometry.py validate.py mesh.py preview.py report.py
  sw/            everything that touches pywin32 lives here
    session.py          COM connection, unit conversion, checked calls
    common.py           axis, blank, sections, loft cut, pattern, measure
    assembly_common.py  insert, place, find entities, mate, check
    bevel_part.py       bevel blank outline and dimension plan
    bevel_assembly.py   coincident apexes, axes at the shaft angle
    spur_part.py        spur blank, and the guide curve for helical teeth
    spur_assembly.py    parallel axes at a centre distance
tools/           standalone drivers and API probes, one per question asked
tests/           pure-Python; SOLIDWORKS is never involved
```

One sub-package per gear type; everything the two share sits at the `gears/`
level. The split was made by moving code, not by rewriting it, so the comments
in `sw/common.py` are the bevel builder's hard-won ones and the measurements in
them were taken on the anchor bevel pinion. They are no less true of a spur
gear.

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

---

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

---

## The SOLIDWORKS build

`build_gear` in [gears/sw/bevel_part.py](gears/sw/bevel_part.py) runs five steps:

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

**The front face is held by a relation, not a dimension.** Its dimension would
measure zero and a zero-length dimension cannot be created. That is not a
workaround: "the front face sits at z = 0" is a statement about the coordinate
system, not a number anyone would edit.

**Helical teeth need a guide curve and straight teeth do not.** Two identical
sections loft to a prism, which is the tooth. Two *rotated* sections loft to a
ruled surface that meets the helicoid only at the ends and cuts inside it in
between, because a loft carries each profile point along a straight line and a
helix is not straight. The guide is a sampled 3D spline rather than an
`InsertHelix` feature — no agreement with SOLIDWORKS about pitch, start angle or
hand, which is three chances to be off by a sign on a part where a sign error is
a gear that will not mesh. It runs through the cap's centreline vertex, and
`split_cap` puts a real vertex there, because a guide that only passes *near* a
spline is the classic way a guided loft fails.

`build_spur_set` in [gears/sw/spur_assembly.py](gears/sw/spur_assembly.py)
places the pair the same way — transforms first, mates after — but on parallel
axes a centre distance apart:

```
position    pinion: origin coincident with the assembly origin        (3)
            gear:   axis at distance a from the pinion axis           (1)
axis        component axis coincident with the assembly Top plane     (1)
direction   pinion: axis also coincident with the Right plane         (1)
            gear:   axis also coincident with the Front plane         (1)
```

The gear's position is a distance mate rather than a coincident origin, because
the two origins are `a` apart instead of sharing an apex. Its direction is a
second plane coincidence rather than an angle mate, because every axis here is
parallel to +Z so the Front plane serves — and an angle mate at zero degrees
solves to 180 as readily as to 0. This is also the first caller of
`to_array_data`'s translation argument; a bevel pair always passed zero.

**Two answers are still open, and both need a seat in front of SOLIDWORKS.**

`MARK_LOFT_GUIDE = 2` is the one constant in `session.py` not read out of
`swconst.tlb` — guide curves have no enum, so 2 is only what the API reference
gives. [tools/probe_helix_loft.py](tools/probe_helix_loft.py) settles it and
three other questions the documentation will not: whether a sampled spline is
accepted as a guide at all, whether a guided cut still survives the circular
pattern with `GeometryPattern` on, and whether the guide may overrun the
profiles. It measures the volume removed against an unguided control, because a
mark SOLIDWORKS ignores hands the control's number straight back. Its fallback,
if a guided cut is refused outright, is to loft through N intermediate sections
instead — the profile generator is identical either way.

The **gear mate's Reverse sense has not been measured for a spur pair**, and the
bevel answer does not transfer: those axes stand at 90° to each other and these
are parallel. `--reverse-gear` is there from the first build. Drag the pinion,
watch which way the gear goes, and record the answer in the module docstring of
`gears/sw/spur_assembly.py`.

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

`tools/smoke_com.py`, `tools/probe_dimension.py`, `tools/probe_mate.py`,
`tools/probe_gear_sense.py` and `tools/probe_helix_loft.py` exist to answer API
questions in isolation before trusting an answer inside a build. Add to them
rather than debugging inside a builder.

---

## Testing

653 tests, all pure Python, all fast. They are closed-form checks on the
geometry — cone distances agreeing between members, tooth tips landing on the
mate's back cone, centre distance solved two independent ways, loft sections
clearing the blank, the guide helix touching a vertex that exists in both
profiles, the gear ratio solved back out of the rolling condition — plus the
validators, the preview arithmetic, the CLI dispatch and the GUI's parameter
plumbing.

The rule that makes them worth having: **express each COM precondition as a pure
Python property of the geometry.** `test_the_guide_helix_touches_the_cap_vertex_of_both_end_sections`
exists because a guided loft fails when its guide only passes *near* a spline,
and that is cheaper to assert here than to discover in SOLIDWORKS.

`sw/` is not covered, because it needs a CAD seat. Anything in there is verified
by building the anchor set in SOLIDWORKS and looking at it. **Carl does that
verification, and his hands-on result outranks any API probe.** If you change
`sw/`, say plainly that you have not run it.

### The MCP as an inspection surface

There is a SOLIDWORKS MCP server configured in `.mcp.json`. It does not build
these gears — it has no mate or component-insert operations at all, so an
assembly is beyond it — but it is a good way to *look at* what the builders
produced without leaving the terminal: `list_bodies` and `body_volume` against
the closed-form blank volume, `probe_section` and `section_screenshot` on a
transverse plane compared with the Python profile at the same z, `check_clearance`
between two components, `take_screenshot` for the visual.

That is evidence, not proof, and it ranks below Carl looking at the part.

---

## House style

The comments in this codebase carry *why*, not *what*, and they are unusually
long where the reason is unusually subtle. Several record measurements —
"measured on the anchor pinion, setting CrownRadius to 22 mm walks the crown
from (19.9220, 22.5585) to (22.0000, 21.0000)". Keep that: a number someone
measured once is worth more than a paragraph of reasoning about what should
happen. Match the surrounding density when you add code.
