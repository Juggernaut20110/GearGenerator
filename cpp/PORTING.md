# C++ port architecture

This document records the architecture observed on `master` and the native
boundary chosen on `feature/cpp-port`. The Python implementation remains the
behavioral reference. This stage establishes contracts and a buildable native
skeleton; it deliberately does not translate the gear solvers line by line.

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

`geargen_preview` currently contains only value-oriented scene models and
export. A later Qt painter/widget adapter can consume those scenes without
forcing `QPointF`, `QMatrix4x4`, `QString`, or QObject ownership into core.
Likewise, `geargen_solidworks` currently exposes a backend/session boundary;
the future Windows COM implementation will live behind it.

## Native module mapping

The first skeleton maps the conceptual modules as follows:

| Python concept | Native location | Porting rule |
| --- | --- | --- |
| `Point2`, `Point3`, matrix helpers | `src/core/common/geometry_types.hpp` | Plain doubles; no Qt or CAD handles. |
| Four parameter dataclasses | `src/core/common/parameters.hpp`, `src/core/{bevel,spur,hypoid,planetary}` | Value records first; JSON is an application-boundary concern. |
| `JsonParams` | `src/core/common/serialization.hpp/.cpp` plus `tools/cpp_reference` | Native emission is deterministic and dependency-free; file loading/migration remains an application-boundary task. |
| `Issue`, `ValidationResult` | `src/core/validation` | Preserve ordered errors/warnings and the non-throwing validation contract. |
| `involute.py` | `src/core/involute` | Preserve named profile segments and analytical parameters before sampling. |
| `placement.py` and `*/mesh.py` | `src/core/placement`, then per-family mesh modules | Keep matrix convention and clocking tests independent of preview/CAD. |
| `*/geometry.py` | Per-family core directories | Port derived records and equations in small behavioral slices, not file order. |
| `preview.py` | `src/preview/scene2d` | Scene data is renderer-neutral. |
| `preview3d.py` | `src/preview/scene3d` | Keep section sampling and camera math independent of Qt. |
| DXF/CSV | `src/preview/export` | Export exactly the same named loops/points that the builder consumes. |
| `sw/session.py` | `src/solidworks` | COM ownership, call checking, VARIANT/SAFEARRAY helpers and unit conversion end at this boundary. |
| `gui.py` | `src/gui` | Qt owns controls, event loop, workers and presentation; it calls core/preview interfaces. |

The type-specific `.cpp` files now implement the parameter records, the
size-dependent `with_defaults` behavior, angle/transverse accessors, compact
derived scalar projections, and the first validation rules. They intentionally
do not claim that tooth-space generation or full hypoid Method 1 geometry has
been ported.

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
reference's named sampling limits such as `MAX_SECTION_SAGITTA_MM = 0.02` and
`FLANK_POINTS = 40` will become separate geometry contracts; they are not
silently substituted by renderer tessellation settings.

## Preview and export architecture

The 2D preview should port as a pipeline:

1. Core computes named tooth-space segments and the blank outline.
2. A type-specific preview assembles tooth/space/reference polylines into a
   renderer-neutral scene.
3. The Qt adapter draws the scene and owns pan/zoom/input events.
4. Export consumes the same scene/profile points. DXF/CSV must never recreate
   geometry from display pixels.

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
  restoration of global sketch/dimension flags through RAII.
* Late binding needs explicit SAFEARRAY/VARIANT construction, method/property
  disambiguation, checked return values, and by-reference error/status reads.
* CAD units are SI metres/radians even though the core is mm/radians and the
  UI lengths are mm.
* Parts are created from fully dimensioned blanks, tooth-space loft cuts and
  circular patterns. The guide curve and section sampling are geometry
  decisions, not UI decisions.
* Assembly components are placed by arithmetic transform first, unfixed, and
  constrained afterward. Mates verify the already-correct placement and
  provide articulation; they are not the primary placement solver.
* SOLIDWORKS accepts a finite spanning tree of mates for an assembly. A
  planetary train must not add both meshes for every planet if that
  over-defines the assembly.

The C++ GUI will use a worker object/thread and queued value-only result
messages. The real COM session will be constructed inside that worker. Build
errors, measurements, paths and mate summaries cross the thread as copied
records/strings only.

## Planned port order

1. Keep this branch's CMake/Qt smoke application and native smoke tests green.
2. Port common types, constants, validation records and parameter schemas;
   add Python-vs-C++ JSON/default fixtures.
3. Port involute primitives and tooth-space segment topology, beginning with
   the external straight spur anchor and its internal counterpart.
4. Port spur derived geometry, backlash modes, profile shifts, active limits,
   section sampling, and placement/clocking; compare every named field.
5. Port 2D scenes and DXF/CSV, then the sampled 3D scene/camera.
6. Port planetary as a composition of the verified spur core: ring derivation,
   station arithmetic, clocking, and train interference invariants.
7. Port bevel straight geometry, then CrownTrace/spiral/Zerol sections and
   cone placement.
8. Port the hypoid Method 1 solver only after its numerical fixtures and
   contact/offset conventions are fixed.
9. Port SOLIDWORKS part primitives and one spur pair, using a live CAD seat;
   then bevel, planetary and hypoid builders.
10. Replace the Qt smoke UI with real parameter/readout/preview wiring and
    keep the Python implementation available until parity gates pass.

## Compatibility strategy and tolerances

The Python implementation is the oracle. `tools/cpp_reference/export.py`
currently emits ten compact fixtures covering external/helical/internal spur,
straight/spiral/Zerol bevel, a hypoid offset sample, planetary valid/edge
cases, and an invalid-input case. The committed JSON keeps `inputs`, ordered
`validation`, `derived`, and a reserved `geometry` object. Large point arrays
are intentionally deferred until the corresponding native profile topology is
ported. The fixture format preserves ordering and can represent null for
non-finite values.

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

`geargen_reference_tests` parses those snapshots and compares native scalar
results numerically; it does not compare formatted floating-point strings.
Validation fields/messages are exact. The existing `geargen_tests` continues
to cover the shared involute scalar, clocking, column-major transform packing,
scene bounds, DXF structure, and the mm-to-m CAD boundary.

## Known risks and open questions

* Native preset loading is still open. The current serializer is intentionally
  output-only so JSON emission and reference snapshots do not pull Qt or a JSON
  dependency into core; unknown-key filtering and type-specific migrations
  still need a checked parser at the GUI/file boundary.
* Python uses double precision and stable `math` operations; compiler flags,
  fused multiply-add and libm differences need measured tolerances rather than
  assumed bit identity.
* Python's banker rounding in a few default/clocking paths must be reproduced
  intentionally where it affects a boundary case.
* Root fillets, generated roots, internal flanks and undercut start-of-involute
  intersections are the highest-risk topology ports. A profile that merely
  looks similar is not compatible.
* Bevel crown trace signs, Zerol hand at zero mean angle, and hypoid signed
  contact azimuth are convention traps.
* Section sample count is derived from sagitta limits and must not be chosen by
  the eventual 3D renderer.
* A native Windows COM implementation can be built without a CAD installation,
  but meaningful part/assembly verification still requires SOLIDWORKS and a
  matching template/configuration.
* The spur external gear Reverse flag, internal pair Reverse flag, and
  catalogue naming of bevel hand remain measured/live-CAD questions in the
  reference project; do not infer them from a convenient sign.
* Qt was not discoverable in the current environment. CMake therefore makes
  the GUI conditional while preserving a real Qt Widgets target when Qt 5/6 is
  installed.

## Current stage acceptance

The native foundation stage is complete when `cpp/CMakeLists.txt` configures
with the verified Visual Studio 2022/MSVC generator, the dependency-free
libraries plus `geargen_tests` and `geargen_reference_tests` build, the Python
suite and CTest pass, and a Qt-enabled configuration builds `geargen_gui` and
opens `GearGenerator`. No full tooth-profile or CAD parity claim is made by
this stage. The largest remaining behavioral gaps are the complete spur
involute/root geometry, bevel CrownTrace sections, the hypoid Method 1
non-zero-offset solver, and all native preview/SOLIDWORKS construction bodies.
