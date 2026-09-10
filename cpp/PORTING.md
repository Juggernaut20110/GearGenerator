# C++ port architecture

This document records the architecture observed on `master` and the native
boundary chosen on `feature/cpp-port`. The Python implementation remains the
behavioral reference. The native core now includes the common numerical
foundation, analytical cylindrical involute/root profiles, complete spur pair
derivation, and planetary composition; it remains independent of Qt and
SOLIDWORKS. The native Qt Widgets application now provides the first usable
parameter, validation, derived-value, preview, preset, and export workflow.

## Python architecture discovered

The Python tree is already split by responsibility rather than by GUI screen:

| Area | Reference modules | Responsibility |
| --- | --- | --- |
| Inputs and presets | `gears/*/params.py`, `params_io.py` | Frozen parameter records, public units, defaults, JSON migration and tolerant unknown-key loading. |
| Shared math | `involute.py`, `placement.py` | Planar involute and generated-root calculations, internal flank and fillet helpers, 3x3 transforms, ratios and clocking. |
| Validation | `validate.py`, `*/validate.py` | Accumulating errors and warnings without raising; type-specific geometry checks run only after basic checks pass. |
| Gear geometry | `bevel/geometry.py`, `spur/geometry.py`, `hypoid/geometry.py`, `planetary/geometry.py` | Derived member/set records, pitch and working dimensions, profile limits, sections, tooth-space loops, and sampled 3D guides. |
| Mesh/placement | `*/mesh.py` | Per-type axis arrangement, member transforms, centre/orbit translations, mesh ratios and gear clocking. |
| 2D preview/export | `preview.py`, `*/preview.py` | Tk-independent scene primitives, views, tooth/space boundaries, scene builders, DXF and CSV serialization. |
| 3D preview | `preview3d.py` | Sampled tooth sections and axes, assembly transforms, orthographic camera/orbit/pan/zoom, and a wireframe scene consumed by Tk Canvas. |
| UI | `gui.py` | Tk widgets, field parsing, live debounced recomputation, scene/readout selection, preset dialogs and export commands. |
| SOLIDWORKS | `sw/session.py`, `sw/common.py`, `sw/assembly_common.py`, `sw/*_part.py`, `sw/*_assembly.py` | Late-bound COM session, SI unit conversion, checked API calls, dimension/equation setup, part lofts/patterns, component transforms and mates. |

The application has four gear families:

* Bevel uses Gleason/Tredgold development. The pitch apex is the local origin,
  the back cone is developed into a virtual spur gear, and a `CrownTrace`
  carries straight, spiral, or Zerol tooth phase over the face.
* Spur supports external and internal gears, straight and helical teeth,
  ISO profile shifts, optional rack-generated external roots, multiple backlash
  definitions, and pair-level tip alteration.
* Planetary derives `z_ring = z_sun + 2*z_planet`, builds two spur meshes, and
  adds equal-orbit stations and a phase/assembly-condition check.
* Hypoid solves an ISO/Gleason Method 1 skew-axis pair, then uses the shared
  involute/Tredgold section machinery for an explicitly approximate tooth
  surface.

The latest Python commit adds `preview3d.py`. It is not a CAD solid preview:
it samples authoritative tooth-space sections at several axial/cone
positions, closes the visible tooth boundary, applies the same mesh placement
and ratio arithmetic, then projects polylines through an orthographic camera.

## Native dependency direction

The native tree keeps Qt and SOLIDWORKS out of core headers:

```text
GearGenerator / GUI
       |       \
       v        v
   Preview   SolidWorks adapter
       \       /
        v     v
          Core
           |
           +-- common value types / parameters
           +-- involute and gear-family math
           +-- placement and validation
```

The intended CMake dependency graph is:

```text
geargen_gui -> geargen_preview -> geargen_core
geargen_gui -> geargen_solidworks -> geargen_core
GearGenerator -> geargen_gui
geargen_tests -> geargen_core + geargen_preview + geargen_solidworks
```

`geargen_preview` contains only value-oriented scene models and export. The
Qt painter/widget adapter consumes those scenes without forcing `QPointF`,
`QMatrix4x4`, `QString`, or QObject ownership into core. Likewise,
`geargen_solidworks` is a Windows-only adapter over the late-bound COM ABI;
its public request/result records contain no COM or Qt handles.

## Native module mapping

The first skeleton maps the conceptual modules as follows:

| Python concept | Native location | Porting rule |
| --- | --- | --- |
| `Point2`, `Point3`, matrix helpers | `src/core/common/geometry_types.hpp` | Plain doubles; no Qt or CAD handles. |
| Four parameter dataclasses | `src/core/common/parameters.hpp`, `src/core/{bevel,spur,hypoid,planetary}` | Value records first; JSON is an application-boundary concern. |
| `JsonParams` | `src/core/common/serialization.hpp/.cpp` plus `tools/cpp_reference` | Native emission and flat JSON loading are deterministic, dependency-free, and tolerant of unknown keys. |
| `Issue`, `ValidationResult` | `src/core/validation` | Preserve ordered errors/warnings and the non-throwing validation contract. |
| `involute.py` | `src/core/involute` | Preserve named profile segments and analytical parameters before sampling. |
| `placement.py` and `*/mesh.py` | `src/core/placement`, then per-family mesh modules | Keep matrix convention and clocking tests independent of preview/CAD. |
| `*/geometry.py` | Per-family core directories | Port derived records and equations in small behavioral slices, not file order. |
| `preview.py` | `src/preview/scene2d` | Scene data is renderer-neutral. |
| `preview3d.py` | `src/preview/scene3d` | Keep section sampling and camera math independent of Qt. |
| DXF/CSV | `src/preview/export` | Export exactly the same named loops/points that the builder consumes. |
| `sw/session.py`, `sw/common.py` | `src/solidworks/com.*`, `builder_common.*`, `solidworks.*` | RAII COM ownership, checked late-bound calls, VARIANT/SAFEARRAY helpers, sketch/dimension/equation/loft/pattern primitives, and unit conversion end at this boundary. |
| `sw/*_part.py` | `src/solidworks/part_builder.*` | Recreate revolved blanks, generated 3D tooth-space curves, lofted cuts, offcut removal, and circular patterns for spur, bevel, and hypoid parts. |
| `sw/*_assembly.py`, `assembly_common.py` | `src/solidworks/assembly_builder.*` | Transform-first component placement, family-specific mate order, gear ratios, hypoid offset constraints, planetary spanning-tree mates, rebuild, save, and interference reporting. |
| `gui.py` | `src/gui` | Qt owns controls, event loop, workers and presentation; it calls core/preview interfaces. |

The type-specific `.cpp` files implement the parameter records,
size-dependent `with_defaults` behavior, angle/transverse accessors, derived
dimensions, analytical involute/root geometry, section sampling, placement,
clocking, and planetary mesh composition. Bevel now includes the exact
Tredgold/Gleason section construction and CrownTrace phase model. Hypoid now
includes the non-zero-offset ISO Method 1 curvature closure, depth/thickness
calculation, boundary spiral transport, skew-axis phase integration, and its
independent Tredgold section approximation. The native hypoid section remains
the same explicitly approximate Tredgold surface as Python; it is not claimed
to be a full cutter-envelope solver.

## Parameters, defaults, and validation

Public parameters retain the Python distinction between user-facing and
derived data:

* Lengths are millimetres at the core boundary.
* User-entered angles remain degrees in parameter records. Geometry converts
  them once to radians; no formula should repeatedly convert a UI value.
* Bevel module is the outer transverse module. Its defaults depend on pitch
  cone distance and use the tighter curved-tooth face limit for spiral/Zerol.
* Spur and planetary module/pressure angle are normal quantities. The
  transverse conversion is `m_t = m_n/cos(beta)` and
  `alpha_t = atan(tan(alpha_n)/cos(beta))`.
* Hypoid module is the wheel outer transverse module; its Method 1 solver
  derives the mean quantities.
* Planetary ring teeth are derived, never accepted as an independent input.

Default generation must remain deterministic and must be tested separately
from validation. Validation is an ordered report, not an exception path: it
must collect basic errors, type-specific errors, and warnings while preserving
field names/messages used by the UI and reports.

The native validation result preserves ordered `errors`, `warnings`, and a
separate `advisories` channel. The current family validators cover the basic
Python rules and the first internal spur/planetary interference relationship;
full generated-root, active-profile, bevel trace, and hypoid solver checks are
still gated from preview/CAD use until their geometry ports land.

## Numerical and geometric conventions

These conventions are compatibility-critical:

* Core geometry uses millimetres and radians internally. SOLIDWORKS receives
  metres only through the adapter boundary.
* A gear's local axis is `+Z`; a tooth **space** is centred on local `+X`.
  The space is the cutter loop. A rendered tooth is built from adjacent spaces.
* Bevel parts use the pitch apex as origin and extend toward `+Z` along the
  local axis. The gear member is rotated about `Y` by shaft angle; both apexes
  remain coincident.
* Spur parts use parallel `+Z` axes. The pinion remains at the origin and the
  gear is translated along `+X` by working centre distance. An internal ring
  uses the same axis arrangement but reverses the mesh sense.
* Hypoid members are skew axes. The signed offset, contact plane and contact
  azimuths are solved; an arbitrary “looks close” transform is not acceptable.
* Planetary sun/ring axes are concentric, planets use orbit stations
  `a*(cos(theta), sin(theta), 0)`, and the assembly condition is checked rather
  than silently changing a station.
* `gear_clocking(z)` places the gear tooth centre opposite the pinion space at
  the contact line. The residual is wrapped and snapped near zero so a whole
  pitch is never returned for an odd count.
* External meshes have angular ratio `w1/w2 = -z2/z1`; internal meshes have
  `+z2/z1`. The ratio magnitude sent to SOLIDWORKS is positive and the sense is
  a separately measured Reverse flag.
* SOLIDWORKS transform arrays are column-major: the first three values are the
  transformed local X basis, followed by Y and Z, then translation in metres.

The shared scalar comparison contract is `Tolerance{absolute=1e-9,
relative=1e-12}` in `core/common/numerics.hpp`. Length tolerances are in mm and
angle tolerances are in radians because those are the core units. The Python
reference's named sampling limits such as `MAX_SECTION_SAGITTA_MM = 0.02`
and `FLANK_POINTS = 40` are separate geometry contracts; they are not
silently substituted by renderer tessellation settings.

The involute port keeps `inv(alpha) = tan(alpha) - alpha`, samples the curve
in roll parameter, and retains the Python distinction between an external
space, an internal space, and a generated rack root. The rack root is an
analytical rolling envelope; the start of involute is solved by a bounded
intersection search before sampling. Circular fillets are used only for the
legacy path or when the analytical generated-root path does not apply.

## Preview and export architecture

The 2D preview should port as a pipeline:

1. Core computes named tooth-space segments and the blank outline.
2. A type-specific preview assembles tooth/space/reference polylines into a
   renderer-neutral scene.
3. The Qt adapter draws the scene and owns pan/zoom/input events.
4. Export consumes the same scene/profile points. DXF/CSV must never recreate
   geometry from display pixels.

The native implementation now has four renderer-neutral 2D builders under
`src/preview/scene2d`: spur (`transverse`, `twist`, `blank`), bevel
(`developed`, `axial`, `trace`, `blank`), hypoid (`contact`, `section`,
`blank`), and planetary (`train`, `transverse`, `blank`). They use the same
named involute segments as the core for both the visible tooth and the cut
boundary. `Scene2D::bounds`, `View2D`, and the style/legend records contain no
Qt types. The DXF writer follows the Python R12 contract (`AC1009`, millimetre
units, `POLYLINE`/`VERTEX`/`SEQEND`, one layer per style); CSV writers preserve
the Python family-specific headers and coordinate columns.

The native `src/gui/preview_widget.*` adapter owns only presentation state:
it paints `Scene2D` polylines with `QPainter`, supports fit, pan, wheel zoom
around the cursor, and double-click fit, and never regenerates gear geometry.
The main window keeps field metadata separate from the family parameter
records. A single debounced `QTimer` drives parse -> validation -> derivation
-> scene rebuilding, while invalid inputs clear the derived table and disable
the SOLIDWORKS action. Preset and export actions call the existing native
value-based APIs.

The 3D preview follows the Python implementation: section samples come from
the core section function, are placed by the per-family mesh module, and are
returned as a renderer-neutral `Scene3D`. Camera operations are UI-neutral;
Qt/OpenGL is optional and is not part of this stage.

## SOLIDWORKS and threading

The Python COM design contains several constraints that the native port must
keep explicit:

* COM is initialized and uninitialized on the same worker thread that calls
  SOLIDWORKS. The UI thread never receives a COM object.
* A session owns visibility, `CommandInProgress`, document lifetime and
  restoration of global sketch/dimension flags through RAII. `com::Apartment`,
  `com::Ptr<T>`, `com::Bstr`, `com::Variant`, and `com::Dispatch` own their
  corresponding Windows resources.
* Late binding needs explicit SAFEARRAY/VARIANT construction, method/property
  disambiguation, checked return values, and by-reference error/status reads.
* CAD units are SI metres/radians even though the core is mm/radians and the
  UI lengths are mm.
* Parts are created from fully dimensioned blanks, tooth-space loft cuts and
  circular patterns. The guide curve and section sampling are geometry
  decisions, not UI decisions. `part_builder.cpp` shares the common sequence
  while retaining family-specific section and placement conventions.
* Assembly components are placed by arithmetic transform first, unfixed, and
  constrained afterward. Mates verify the already-correct placement and
  provide articulation; they are not the primary placement solver.
* SOLIDWORKS accepts a finite spanning tree of mates for an assembly. A
  planetary train must not add both meshes for every planet if that
  over-defines the assembly.

The current GUI performs native geometry recomputation synchronously after a
short debounce; the ported calculations are fast enough that introducing a
worker would add interaction and lifetime complexity without a measured
benefit. The `Build in SOLIDWORKS` action is different: it starts a `QThread`
worker, and `solidworks::build(BuildRequest)` constructs and destroys the
COM apartment/session entirely on that worker. Only the copied
`BuildResult`—errors, measurements, paths, mate summaries, and interference
data—returns to the GUI through a queued invocation. The GUI never stores or
touches a worker-created COM wrapper.

The native adapter deliberately uses `IDispatch` late binding rather than a
generated SOLIDWORKS type-library wrapper. This keeps ordinary builds free of
the SOLIDWORKS SDK and lets the checked-in CMake project compile on a Windows
machine with only the Windows SDK. The tradeoff is that the subset of enum
values and method signatures used by the Python reference is maintained in
`solidworks.hpp`, and live integration tests still require a compatible
SOLIDWORKS installation, default part/assembly templates, and a license.
`geargen_solidworks_tests` tests COM apartment and VARIANT/SAFEARRAY/BSTR
marshalling without launching SOLIDWORKS.

## Planned port order

1. **Complete:** keep this branch's CMake/Qt application and native tests green.
2. Port common types, constants, validation records and parameter schemas;
   add Python-vs-C++ JSON/default fixtures.
3. Port involute primitives and tooth-space segment topology, beginning with
   the external straight spur anchor and its internal counterpart.
4. Port spur derived geometry, backlash modes, profile shifts, active limits,
   section sampling, and placement/clocking; compare every named field.
5. **Complete:** port 2D scenes, DXF/CSV, presets, and the sampled 3D
   scene/camera.
6. **Complete:** port planetary as a composition of the verified spur core: ring derivation,
   station arithmetic, clocking, and train interference invariants.
7. Port bevel straight geometry, then CrownTrace/spiral/Zerol sections and
   cone placement. **Complete on `feature/cpp-port`.**
8. Port the hypoid Method 1 solver only after its numerical fixtures and
   contact/offset conventions are fixed. **Complete on `feature/cpp-port`.**
9. **Complete:** port SOLIDWORKS COM session/RAII, shared part primitives,
   spur/internal/helical, bevel, hypoid, and planetary part/assembly builders.
   Live-seat body/mate verification remains an environment-dependent gate.
10. **Complete for non-CAD presentation:** replace the Qt smoke UI with
    metadata-driven parameter/readout/preview wiring, presets, and exports;
    keep the Python implementation available until parity gates pass.
11. **Complete for the native automation boundary:** connect the validated GUI
    action through the COM worker boundary. Run live-seat integration fixtures
    and expand recorded invocation coverage when SOLIDWORKS is available.

## Compatibility strategy and tolerances

The Python implementation is the oracle. `tools/cpp_reference/export.py`
currently emits eleven fixtures covering external/helical/internal/profile-
shifted/rack-generated spur, straight/spiral/Zerol bevel, a non-zero-offset
Method 1 hypoid sample, planetary valid/edge cases, and an invalid-input case.
The committed JSON keeps `inputs`, ordered `validation`, `derived`, and a
compact `geometry` object. Bevel and hypoid fixtures include solver scalars,
phase values, cone-distance station lists, and representative named-section
loops rather than unbounded tessellations. The fixture format preserves
ordering and can represent null for optional solver diagnostics.

Comparison policy:

* Validation field order and message text are exact unless a deliberate native
  wording change is recorded.
* Dimensionless counts, enum states, topology labels and member ordering are
  exact.
* Linear values use absolute/relative tolerances stated per fixture in mm;
  angular values use radians. CAD-facing metres are compared after converting
  back to mm.
* Profile/section points use a combined coordinate tolerance and a topology
  check (same segment names, loop orientation, closure and sample count).
* Clocking and placement are checked both as matrix entries and as geometric
  invariants: axis angle, centre distance, contact point and mesh ratio.
* Preview scenes compare scene keys, titles where user-visible, styles,
  polyline count/order and points. A visual screenshot alone is insufficient.
* SOLIDWORKS compatibility is checked first through pure preconditions, then
  with a live seat: body count, measured dimensions, axes, mate status,
  articulation and interference. The live result outranks an approximate
  renderer preview.

`geargen_reference_tests` parses those snapshots and compares native scalar,
section, phase, solver, and loop results numerically; it does not compare
formatted floating-point strings. Validation fields/messages are exact where
the current fixture exercises them. The existing `geargen_tests` covers the
shared involute scalar, cylindrical geometry, bevel CrownTrace/sections,
hypoid Method 1 convergence and zero-offset behavior, clocking,
column-major transform packing, scene bounds, DXF structure, and the mm-to-m
CAD boundary.

## Known risks and open questions

* Preset migration hooks beyond Python's current unknown-key tolerance remain
  open. The native flat-object parser intentionally covers the present JSON
  schema without pulling Qt or a third-party JSON dependency into core; a
  future versioned migration layer should be added if the file schema becomes
  nested.
* Python uses double precision and stable `math` operations; compiler flags,
  fused multiply-add and libm differences need measured tolerances rather than
  assumed bit identity.
* Python's banker rounding in a few default/clocking paths must be reproduced
  intentionally where it affects a boundary case.
* Root fillets, generated roots, internal flanks and undercut start-of-involute
  intersections remain the highest-risk topology areas even though the
  cylindrical fixtures now pass. A profile that merely looks similar is not
  compatible.
* Bevel CrownTrace signs, Zerol hand at zero mean angle, and hypoid signed
  contact azimuth are convention traps; the current fixtures cover the anchor
  right-hand cases, while mirrored left-hand and negative-offset combinations
  remain useful expansion cases.
* The native hypoid phase quadrature and terminal loft-clearance search have
  explicit convergence limits. Future diagnostics should carry failing
  interval/iteration details rather than only an exception string.
* Section sample count is derived from sagitta limits and must not be chosen by
  the eventual 3D renderer.
* A native Windows COM implementation can be built without a CAD installation,
  but meaningful part/assembly verification still requires SOLIDWORKS and a
  matching template/configuration. The current CI-safe tests stop at checked
  marshalling and core construction intent; they do not claim that a live
  document was rebuilt.
* The spur external gear Reverse flag, internal pair Reverse flag, and
  catalogue naming of bevel hand remain measured/live-CAD questions in the
  reference project; do not infer them from a convenient sign.
* Qt is available as a MinGW Qt 6 installation while MSVC is provided by the
  Visual Studio 2022 Build Tools. The GUI remains conditional so the verified
  MSVC core build does not depend on mixing toolchains.

## Current stage acceptance

The native SOLIDWORKS stage is complete when the Visual Studio/MSVC
core/adapter build, Qt build where a matching Qt toolchain is installed,
native tests, Python suite, and fixture comparisons pass. This stage now has
the MSVC-buildable COM adapter, all current family construction paths, and the
worker-thread GUI connection. Remaining environment-dependent work is live
SOLIDWORKS verification of measured bodies, mate status, rebuild behavior,
and saved documents against representative templates. The native hypoid
section deliberately retains Python's documented Tredgold approximation
boundary; replacing it with a true cutter-envelope model would be a separate
geometry decision, not a compatibility fix.
