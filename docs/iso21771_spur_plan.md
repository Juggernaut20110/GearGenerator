# ISO 21771 spur/cylindrical gear upgrade plan

Status: engineering audit and implementation plan only. No geometry changes are
included in this document's change.

Audit target: `master` at `3266242` (`Clarify hypoid spiral angle and hand
convention`).

## 1. Scope and source status

This plan covers the external spur, external helical, and internal ring-pair
paths implemented by this repository. The intended standards boundary is:

- [ISO 21771-1:2024](https://www.iso.org/standard/84949.html) for concepts and
  geometry of cylindrical involute gears and gear pairs, including reference
  versus working quantities, profile shift, internal-gear sign conventions,
  tooth engagement, contact ratios, backlash terminology, and generated form
  limits.
- [ISO 53:1998](https://www.iso.org/standard/22643.html) for the standard basic
  rack tooth profile when a rack/tool profile is used: addendum, dedendum,
  clearance, pressure angle, and basic-rack root fillet definition.

ISO 21771-1 is a geometry and terminology standard. It is not a SOLIDWORKS
feature recipe and it does not make the repository's sampling density, loft
construction, blank construction, or validation thresholds normative. ISO
21771-1:2024 also adopts the signed negative-tooth-count convention for internal
gears; this repository currently uses positive tooth counts plus an `internal`
flag. The implementation should preserve that public API and introduce an
explicit conversion boundary rather than expose negative counts to existing
callers.

There are three different categories of statements in this plan:

1. **ISO terminology or geometry:** a quantity is named and defined by
   ISO 21771-1, or the basic rack is taken from ISO 53.
2. **Closed-form implementation equation:** the equation is the physical,
   positive-radius form used by this repository, derived from the ISO concepts
   and the involute geometry. It must be independently checked before code is
   merged.
3. **Implementation choice:** a compatibility alias, a preview convention, a
   validation threshold, a default, a sampling method, or a SOLIDWORKS strategy.

The public ISO pages establish publication and scope, but do not expose all of
the paid standard text. A public preview of ISO 21771-1 confirms the relevant
headings, symbols, internal sign convention, and the working-pressure-angle
relations. The exact complete text of the formulas in the tooth-thickness,
generated-root, and internal-cutter portions was not available from an
authoritative ISO source during this audit. Those parts are explicitly marked
below and must be checked against a licensed copy before implementation claims
conformance.

## 2. ISO terminology mapped to this repository

The ISO symbol `z` is signed for internal gears in ISO 21771-1. In the table,
`Z` means the repository's positive magnitude (`p.z2` for a ring).

| ISO concept | Repository today | Planned meaning |
|---|---|---|
| `m_n` normal module | `SpurSetParams.module`, `p.module` | Keep as the user-facing module and the rack/tool scale. |
| `m_t` transverse module | `p.transverse_module`; `SpurSetGeometry.transverse_module` | Keep as `m_n / cos(beta)` and expose it as a named derived quantity. |
| `z` tooth count | `p.z1`, `p.z2`, both positive | Keep the API positive. Convert to `z_iso = -Z` only inside ISO-style internal equations. |
| `alpha_n` normal pressure angle | `p.alpha_n`; input `pressure_angle` is degrees | Keep. This is the rack/tool angle for the helical case. |
| `alpha_t` transverse pressure angle | `p.alpha_t`; `geo.transverse_pressure_angle` | Keep as `atan(tan(alpha_n) / cos(beta))`. |
| `beta` helix angle | `p.beta` is signed by `hand`; each member has signed `beta` | Keep the signed member convention. Document that the pair's hand rule is a mesh/placement choice, not an ISO profile-shift quantity. |
| `d` reference diameter | `2 * member.pitch_r` | Rename internally to `reference_d`; keep `pitch_r` as a deprecated compatibility alias for the reference radius. |
| `d_b` base diameter | `2 * member.base_r` | Keep, but derive from the reference circle and `alpha_t`, never from a working circle. |
| `d_a` tip diameter | `2 * member.tip_r` | Keep as the nominal physical tip diameter, with the internal ring's tip being the smaller tooth radius. Add signed ISO adapters and, later, active/form diameters. |
| `d_f` root diameter | `2 * member.root_r` | Keep as nominal root diameter. Do not use it to mean a generated root diameter. |
| `d_fE` / generated root diameter | absent | Add a separate generated/form-root quantity when rack or cutter-envelope generation is implemented. |
| profile shift coefficient `x` | absent | Add `profile_shift1` and `profile_shift2`, dimensionless, with zero defaults. |
| `h_aP*` | `ADDENDUM_FACTOR = 1.0` in `geometry.py` | Add a rack addendum coefficient; default `1.0`. |
| `h_fP*` | `DEDENDUM_FACTOR = 1.25` in `geometry.py` | Prefer an exposed clearance coefficient with `h_fP* = h_aP* + c_P*`; retain a derived dedendum coefficient for reporting. Default clearance `0.25`. |
| `c_P*` | implicit difference `1.25 - 1.0` | Add as an explicit basic-rack clearance coefficient. Validate that it is non-negative and consistent with the selected rack profile. |
| `rho_fP*` | absent; `fillet_factor = 0.2` is a gear-root approximation | Add `basic_rack_root_radius_factor`, with the ISO 53 Profile A candidate default `0.38` in generated-root mode. Do not reinterpret the existing `fillet_factor` silently. |
| `s` / tooth thickness | local `tooth_thickness` in `compute_set`; not stored | Add per-member reference tooth thickness as derived geometry. Keep the old local behavior as the zero-shift compatibility case. |
| `s_n` normal tooth thickness | absent | Add a derived normal thickness, with the normal/transverse conversion explicit. |
| `s_t` transverse tooth thickness | local `tooth_thickness` only | Add a derived transverse reference thickness, including profile shift and the existing symmetric backlash policy. |
| reference circle/cylinder | `member.pitch_r` | Make this the unambiguous reference circle. It is fixed by module and tooth count, not by working centre distance. |
| base circle/cylinder | `member.base_r` | Keep as the involute base circle. |
| tip circle/cylinder | `member.tip_r` | Keep as the tooth-end circle, with directional labels for an internal gear. |
| root circle/cylinder | `member.root_r` | Keep as the nominal root circle; add generated/form-root data separately. |
| working pitch circle/cylinder | absent | Add `working_r`/`working_d`, derived from `alpha_wt` and the base radii. |
| working pressure angle `alpha_wt` | absent; contact uses `geo.transverse_pressure_angle` | Add as a pair-level quantity. It equals `alpha_t` only for the default zero-shift/reference-centre case. |
| reference centre distance | `p.centre_distance` and `geo.centre_distance` | Split into `reference_centre_distance` and `working_centre_distance`. The former is derived from reference diameters. |
| working centre distance `a_w` | absent; `geo.centre_distance` is used by mesh and mates | Make `working_centre_distance` the assembly/mate distance. Keep `centre_distance` as a compatibility alias to it. |
| profile shift | absent | Use ISO-positive external shifts. For the internal ring, document the physical positive-ring convention and convert it to ISO signed equations at the boundary. |
| centre-distance modification | absent | Add derived `centre_distance_modification = (a_w - a_ref) / m_n`; do not identify it with `x1 + x2` or `x2 - x1` except where an equation explicitly proves that special case. |
| transverse contact ratio `epsilon_alpha` | `geo.transverse_contact_ratio` | Keep the public name. Recompute from working pressure angle and active tip/form limits. |
| overlap ratio `epsilon_beta` | `geo.axial_contact_ratio` | Add an ISO-named alias `overlap_ratio`; retain the old property and report label. |
| total contact ratio `epsilon_gamma` | `total_contact_ratio` | Keep and add the ISO alias. |

## 3. Current implementation audit

### Parameters and conversions

`gears/spur/params.py` is a frozen, JSON-serializable dataclass. The current
user inputs are module, tooth counts, face width, bore, hub, normal pressure
angle, helix angle, hand, internal arrangement, a heuristic root-fillet factor,
backlash, and ring rim thickness. The important derived properties are:

- `transverse_module = module / cos(beta)`;
- `alpha_t = atan2(tan(alpha_n), cos(beta))`;
- `centre_distance = m_t * (z1 + z2) / 2` externally and
  `m_t * (z2 - z1) / 2` internally.

The conversions are ISO-compatible for a parallel-axis helical pair. The centre
distance is the reference/nominal distance only because the model has no profile
shift or working geometry. `alpha_t` is a transverse reference pressure angle,
not a working pressure angle.

`JsonParams` already filters unknown JSON keys and allows dataclass defaults to
fill fields absent from old files. That is a strong compatibility seam. New
fields should be additive, and any rename should use
`SpurSetParams._migrate_json_data` rather than changing `JsonParams` globally.

### Geometry and involute profile

`gears/spur/geometry.py` contains the entire derived model in `compute_set`.
The current path is:

1. `pitch_r = m_t*z/2` and `base_r = pitch_r*cos(alpha_t)`.
2. Addendum is `1.0*m_n`; dedendum is `1.25*m_n`.
3. External tip/root radii are `pitch_r + addendum` and
   `pitch_r - dedendum`. Internal ring radii are `pitch_r - addendum` and
   `pitch_r + dedendum`.
4. Tooth thickness is `pi*m_t/2 - backlash/2` for each member. The external
   involute uses tooth thickness; the internal involute uses the complementary
   space width.
5. The pair centre distance is the reference distance. Contact uses the
   reference pressure angle and nominal tip radii.

The first four items are compatible with ISO geometry only in the default
unshifted, standard-rack case. They are not sufficient for arbitrary `x`,
working centre distance, non-standard rack proportions, or exact generated root
form.

`gears/involute.py` correctly isolates the pure involute and has a deliberate
external/internal split. `flank_points` uses a radial segment below the base
circle; its own docstring calls this a simplification of the cutter-dependent
trochoid. `internal_flank_points` requires the ring tip to clear its base circle
and reverses the radial direction. `root_fillet` then fits a circular arc based
on the repository's `fillet_factor`, not on `rho_fP*` and not on a generated
rack envelope. This is an approximation, not a claim that the standard root
is a circular arc of the final gear with that radius.

The current helical twist is `face_width*tan(beta)/pitch_r`. The opposite-hand
external rule and same-hand internal rule are exercised by tests and should be
retained. The section-count/sagitta logic, cap overshoot, and guide curve are
SOLIDWORKS implementation choices and are independent of profile shift.

### Contact and validation

`transverse_contact_ratio` uses the standard nominal tip-circle length of action:

```text
external: sqrt(ra1^2-rb1^2) + sqrt(ra2^2-rb2^2) - a*sin(alpha_t)
internal: sqrt(ra1^2-rb1^2) - sqrt(ra2^2-rb2^2) + a*sin(alpha_t)
```

divided by `pi*m_t*cos(alpha_t)`. The internal signs are correct for the
repository's positive-radius/positive-distance convention and are independently
checked in `tests/test_internal_geometry.py`. They are not written in ISO's
negative-diameter convention, and the function has no working pressure angle or
active-root limit.

`validate.py` warns about standard-rack undercut using a closed-form tooth-count
limit, checks contact ratio, helix overlap, blank wall, and top land, and uses a
fixed ten-tooth internal difference as a practical interference guard. The
undercut warning is honest about the radial-root approximation, but profile shift
will require it to become a geometry-dependent check. The internal ten-tooth
rule should remain a conservative fallback until the exact tip/trimming
interference calculation is implemented and independently validated.

### Mesh, preview, reports, and builders

- `gears/spur/mesh.py` translates by `geo.centre_distance`; clocking and gear
  velocity ratio do not depend on profile shift. The translation must become the
  working distance while the pitch-circle tests must use the reference circles.
- `gears/spur/preview.py` draws `pitch_r` as a single pitch circle, reports it as
  “pitch diameter,” and uses base/root/tip circles without distinguishing
  reference, working, nominal root, or generated root. It is otherwise a good
  consumer because all profile points come from `tooth_space_section`.
- `gears/spur/report.py` prints one “centre distance,” “pitch diameter,” and
  “base radius” per member. It must add reference/working labels without
  removing the old rows immediately.
- `gears/sw/spur_part.py` revolves a blank to the current external tip or ring
  rim, loft-cuts the sampled tooth space, and patterns it. Its blank dimension
  plan relies on the outline's point count and H/V segment order. Shifted tip
  and root values must flow through `blank_outline`, not be recomputed in the
  COM layer.
- `gears/sw/spur_assembly.py` mates the axes at `geo.centre_distance`; this must
  use the working distance. Gear clocking remains a reference-tooth phase
  operation, not a working-circle substitution.
- `tools/build_spur.py` and `tools/build_spur_set.py` construct params directly
  and should receive additive flags. Existing defaults and output filenames
  should remain unchanged for the zero-shift legacy profile.
- `gears/gui.py` owns `SPUR_FIELDS`, auto-sizing, parsing, JSON load/save, and
  the derived readout. It needs an advanced ISO section but must not make old
  presets fail because a new optional field is blank.

## 4. Proposed parameter model

The parameter model should separate user intent from derived geometry. The
following additions are recommended; exact public spelling can be finalized in
the implementation phase, but the semantics should not change.

```python
profile_shift1: float = 0.0
profile_shift2: float = 0.0
basic_rack_addendum_factor: float = 1.0
basic_rack_clearance_factor: float = 0.25
basic_rack_root_radius_factor: float = 0.38
working_centre_distance: float | None = None
root_geometry: str = "legacy"
```

The derived rack dedendum coefficient is
`basic_rack_dedendum_factor = basic_rack_addendum_factor +
basic_rack_clearance_factor`. A separate user-facing dedendum input is not
needed initially; exposing both dedendum and clearance invites inconsistent
profiles. If a later API requires both, one must be authoritative and the other
must be derived/validated.

`root_geometry="legacy"` is intentional. It preserves the current radial
below-base-circle segment and `fillet_factor` behavior for old JSON and CLI
invocations. `root_geometry="rack_generated"` opts into a rack/cutter envelope
using `basic_rack_root_radius_factor`; the latter defaults to the ISO 53 Profile
A candidate `0.38`, but it has no effect in legacy mode. This is the only way to
satisfy both goals: improve the model and guarantee that adding fields does not
silently change existing solids.

The old `fillet_factor` remains accepted and serialized. In legacy mode it
continues to mean the existing final-gear circular-fillet approximation. In
generated mode it should either be rejected as ambiguous or explicitly documented
as ignored. Do not silently treat `fillet_factor=0.2` as a basic-rack coefficient.

`working_centre_distance=None` means “derive the working distance from the two
profile shifts.” If a user supplies it, it becomes the assembly distance and
`alpha_wt` is solved from it. The validator must compare the implied shift sum or
difference to the supplied `profile_shift1/2` values and report a conflict rather
than build a non-conjugate pair. This keeps centre distance and profile shift
from becoming two contradictory sources of truth.

Defaults and migration rules:

- all new numeric fields default to the current standard values or zero;
- `working_centre_distance=None` derives the old nominal distance at `x1=x2=0`;
- `root_geometry="legacy"` preserves the current section topology and point
  behavior;
- old JSON loads through dataclass defaults;
- a future rename such as `x1` to `profile_shift1` is handled explicitly in
  `_migrate_json_data` and canonical JSON writes only the new name;
- old CLI invocations do not acquire new required flags or new filename
  components.

## 5. Reference versus working geometry

The most important compatibility rule is: **`pitch_r` must never be silently
repurposed to mean a working radius.**

For each member, the planned derived object should have at least:

```text
reference_r / reference_d     reference circle, d = m_t*|z|
working_r / working_d        operating circle, d_w = d_b/cos(alpha_wt)
base_r / base_d               involute base circle, d_b = d*cos(alpha_t)
tip_r / tip_d                 nominal tooth-end circle, directional for a ring
root_r / root_d                nominal root circle, directional for a ring
generated_root_r               actual generated/form boundary when available
```

The compatibility property `pitch_r` returns `reference_r` and is marked for
deprecation in code comments and API documentation. Existing preview, mesh, and
tests that mean “reference pitch circle” continue to get the same value. New
code must use `reference_r` or `working_r` by name.

At the set level:

```text
reference_centre_distance = reference_r2 + reference_r1       external
reference_centre_distance = reference_r2 - reference_r1       internal
working_centre_distance   = working_r2 + working_r1             external
working_centre_distance   = working_r2 - working_r1             internal
centre_distance            compatibility alias of working_centre_distance
```

The physical radii above are positive. The ISO adapter additionally exposes an
internal ring with negative signed `z`, `d`, `d_b`, `d_a`, `d_f`, and centre
distance where ISO requires that convention. No existing mesh or SOLIDWORKS
caller should receive those signed values.

The tip/root naming used by the current code is useful for geometry but can be
misread in a ring. Keep the physical names and add directional report labels:
“tip radius (inner)” and “root radius (outer)” for the ring. `outside_dia` remains
the diameter occupied by the teeth, not the outer rim diameter.

## 6. Profile-shift equations and sign audit

The equations below are the proposed positive-radius implementation equations.
They intentionally use `Z = abs(z2)` for a ring and branch explicitly for
external/internal pairs. The ISO signed form can then be tested separately by
mapping `z_iso2 = -Z` and signed diameters.

Let

```text
M       = m_n
C       = cos(beta)
m_t     = M/C
A       = h_aP*
F       = h_fP* = h_aP* + c_P*
alpha_t = atan(tan(alpha_n)/C)
```

For an external pair, define `q = z1 + z2` and `X = x1 + x2`. For an internal
pair, define `q = Z - z1` and `X = x2 - x1`, where member 2 is the ring. The
internal `x2 - x1` sign is independently supported by the standard internal
gear design convention: a positive ring shift makes its addendum shallower and
the operating distance relation uses the ring shift minus the pinion shift.
That sign must get a dedicated test; it must not be inferred from the external
branch by changing one plus sign.

### Individual reference geometry

For the external pinion and external gear:

```text
r_i   = m_t*z_i/2
h_ai  = M*(A + x_i)
h_fi  = M*(F - x_i)
r_ai  = r_i + h_ai
r_fi  = r_i - h_fi
r_bi  = r_i*cos(alpha_t)
```

For the internal ring, using positive physical radii:

```text
r_2   = m_t*Z/2
h_a2  = M*(A - x2)       inward tooth addendum
h_f2  = M*(F + x2)       outward tooth dedendum
r_a2  = r_2 - h_a2       inner tip circle
r_f2  = r_2 + h_f2       outer root circle
r_b2  = r_2*cos(alpha_t)
```

The nominal whole tooth depth is `h_a + h_f`; a generated root can stop above
the nominal root circle, so it must not overwrite `r_f` when reporting nominal
ISO dimensions. The physical formula above agrees with the current defaults and
with the independently checked internal sign behavior at `x2=0`.

### Reference tooth thickness

The proposed ideal reference thickness for either member in a transverse
section is:

```text
s_n,i = M * (pi/2 + 2*x_i*tan(alpha_n))
s_t,i = s_n,i / cos(beta)
      = m_t * (pi/2 + 2*x_i*tan(alpha_n))
```

The repository's current backlash input is a circular/transverse allowance at
the reference circle. For compatibility, the first implementation continues
to subtract half from each member:

```text
s_t,i,actual = s_t,i - backlash/2
s_n,i,actual = s_t,i,actual*cos(beta)
```

This is an implementation policy, not a claim that every ISO backlash and
tolerance case is represented. A later tolerance API should distinguish
reference circumferential backlash, transverse backlash, normal backlash, and
working backlash. The internal ring space is formed from the complement of its
transverse tooth thickness at the reference circle, with the same compatibility
backlash policy.

The base-circle half-angle constants then become, for an external member,

```text
psi0 = s_t,actual/(2*r_i) + inv(alpha_t)
```

and, for the internal ring,

```text
psi0 = (pi*m_t - s_t,actual)/(2*r_2) + inv(alpha_t)
```

The existing `psi0` construction is therefore correct at zero shift and must be
extended rather than replaced by a new arbitrary angular offset.

### Working pressure angle and distance

For a profile-shift-derived operating condition, solve the involute equation:

```text
inv(alpha_wt) = inv(alpha_t) + 2*X*tan(alpha_n)/q
```

Then derive the physical working distance:

```text
a_w = (q*m_t/2) * cos(alpha_t)/cos(alpha_wt)
```

Equivalently, when a working centre distance is the authoritative input:

```text
cos(alpha_wt) = (a_w / a_ref) * cos(alpha_t)
implied_X     = q*(inv(alpha_wt)-inv(alpha_t))/(2*tan(alpha_n))
```

The validator compares `implied_X` to `x1+x2` externally or `x2-x1`
internally. The solver must reject an invalid inverse-involute domain and a
working distance that makes `alpha_wt` non-physical.

Working radii and diameters are then:

```text
r_wi = r_bi/cos(alpha_wt)
d_wi = 2*r_wi
```

The internal working distance is `r_w2-r_w1`, not a sum. This is the independent
sign check that should be used in tests and in the mesh placement.

### Contact ratios

For an unmodified nominal involute active to the tip circle, use the working
pressure angle and working distance in the existing positive-radius length of
action:

```text
B_i = sqrt(max(0, r_ai^2 - r_bi^2))

g_alpha_external = B_1 + B_2 - a_w*sin(alpha_wt)
g_alpha_internal = B_1 - B_2 + a_w*sin(alpha_wt)

p_bt = pi*m_t*cos(alpha_t)
epsilon_alpha = g_alpha / p_bt
```

The external/internal sign pair is deliberately retained and independently
tested. The denominator is the reference transverse base pitch for parallel-axis
gears; if the implementation later supports unequal or crossed working base
pitches, it must use the corresponding ISO path-of-contact pitch instead of
silently retaining this shortcut.

When a generated root/undercut or active-profile limit is in effect, the active
root/start diameter must be used to limit the path of contact. In other words,
the formula above is the nominal full-involute formula, not a promise that every
undercut gear has that contact ratio. The implementation phase must decide
whether the active root is represented by an ISO-generated `d_Nf`/form value or
whether that case remains a validation error.

For the helical pair, retain the axial overlap equation:

```text
epsilon_beta = face_width * abs(sin(beta)) / (pi*m_n)
epsilon_gamma = epsilon_alpha + epsilon_beta
```

Expose these as `overlap_ratio` and `total_contact_ratio`, while keeping
`axial_contact_ratio` as a compatibility alias. The signed hand controls twist;
the overlap magnitude uses the helix magnitude.

## 7. Root geometry recommendation

ISO 21771-1 distinguishes the nominal root circle from generated/form geometry
and includes generated-root material in its later geometry sections and Annex B.
ISO 53 defines a basic rack tooth whose root transition is a circular fillet of
radius `rho_fP`; that does not mean the final gear root is the same circle. The
final root is the envelope left by the tool motion.

The current model does not generate that envelope:

- below the base circle, `flank_points` inserts a radial segment;
- `root_fillet` fits an approximate circular arc in the final gear space;
- undercut is warned about but is not represented in the solid;
- the internal ring uses a mirrored involute-space construction and has no
  cutter-generated root model.

**Recommendation: choose B as the target, keep A as the compatibility mode.**

### B: rack/cutter envelope (target ISO mode)

Implement a pure-math profile generator that:

1. builds the selected basic rack/tool profile in a normal section from
   `alpha_n`, `h_aP*`, `h_fP*`, `c_P*`, and `rho_fP*`;
2. rolls the rack/cutter against the gear and samples the envelope in the
   transverse plane;
3. identifies the start of the nominal involute, undercut/trochoid portion,
   generated root diameter, and the transition to the root fillet;
4. supports the external rack/hob case first and an explicit pinion-type
   cutter/shaper case for internal rings;
5. sweeps the transverse result into the existing helical loft sections without
   changing the face-width/hand machinery.

This gives the model a real generated root and makes the undercut warning
meaningful. It also has the largest implementation and verification cost, so it
should be a separately selectable mode until its external and internal tests are
complete.

### A: documented approximation (compatibility mode)

Retain the existing radial-below-base and final-gear fillet path, but label it
explicitly as `legacy` in reports and documentation. It remains useful for
preview/SOLIDWORKS compatibility and for old JSON. It must not be described as
ISO-generated geometry, and validators must continue to warn when the real
generated gear would undercut.

The ISO standard does not, by itself, require this repository to use a specific
trochoidal construction for every CAD preview. The standard defines the
geometrical concepts and gives generated-form calculations; the decision to
implement the tool envelope, a simplified root, or a CAD-native sweep is an
implementation choice. The plan chooses the envelope for the opt-in ISO mode
because it is the closest practical match to the stated goal.

## 8. Compatibility strategy

### Geometry API

- Add explicit `reference_r`, `working_r`, `reference_d`, and `working_d`.
- Preserve `pitch_r` as a read-only alias for `reference_r`.
- Add `reference_centre_distance` and `working_centre_distance` to
  `SpurSetGeometry`; preserve `centre_distance` as an alias of the latter.
- Keep `tip_r`, `root_r`, `base_r`, `addendum`, `dedendum`, `psi0`, and `twist`
  available during migration. Clarify in docstrings whether a quantity is
  nominal, generated, reference, or working.
- Keep `outside_dia` and `rim_radius` physical and positive. Never feed ISO
  signed ring diameters into SOLIDWORKS or preview transforms.

### Mesh, preview, and DXF/CSV

- Mesh translation and the assembly distance mate use `working_centre_distance`.
- Reference-circle overlays continue to use `reference_r`; a new working-circle
  overlay is added rather than replacing the old one.
- Preview titles/readouts say “reference” and “working” explicitly. Existing
  labels remain as aliases for the default reference values until the tests and
  user-facing documentation are migrated.
- Tooth-space and CSV point topology stays unchanged in legacy mode. Generated
  root mode may add named `trochoid`/`generated_root` segments, but the existing
  named segments (`flank`, `riser`, `cap`, `root`) remain available to consumers.
- DXF export continues to export the selected scene, not a hidden signed ISO
  coordinate system.

### SOLIDWORKS

- `blank_outline` remains the only source of blank dimensions. The builder keeps
  its current point count and alternating segment structure in legacy mode.
- The assembly uses the working distance, which equals the old distance at
  defaults. A shifted set therefore gets the correct operating placement.
- Part and assembly filenames remain unchanged for defaults. For non-default
  shifts/rack/root modes, add a stable suffix only after a filename collision
  test is added; otherwise a shifted set can overwrite a standard set with the
  same module and tooth counts.
- Expected radius checks use the physical tip/rim values and remain independent
  of ISO signed display values.
- The helix section-count, guide-curve, end-overshoot, hand, clocking, and
  pattern logic are retained unless a generated root adds more section segments.

### JSON, GUI, and CLI

- `JsonParams`' unknown-key filtering is retained.
- Old JSON gets zero shifts, standard rack coefficients, derived nominal centre
  distance, and legacy root mode through dataclass defaults.
- New JSON writes canonical field names. Migration tests cover both an old file
  with no new keys and any transitional aliases.
- GUI exposes profile shifts and working centre distance in an “Advanced ISO
  geometry” group. Rack coefficients and root mode can initially be advanced
  fields or CLI-only, but the derived readout must show what was actually used.
- Add flags to both spur build tools and the report CLI without changing the
  meaning of existing flags. The old `--beta`, `--alpha`, `--hand`,
  `--backlash`, `--internal`, `--rim`, and sizing flags continue to produce the
  current default geometry.

## 9. Implementation phases

### Phase 0 — freeze the audit and equation fixtures

- Add this plan and a short source/equation checklist to the review.
- Obtain licensed copies or authoritative extracts of ISO 21771-1:2024 and
  ISO 53:1998 before coding the formula-heavy phases.
- Add independent expected-value fixtures for the current external, helical,
  and internal anchor sets.
- Record the current `pitch_r` and `centre_distance` aliases as reference
  quantities, not working quantities.

### Phase 1 — additive parameter model and compatibility aliases

- Add new dataclass fields, validation, JSON defaults, and migration hooks.
- Add derived rack coefficients and an explicit legacy/generated root mode.
- Add `reference_*` and `working_*` names while retaining old properties.
- Do not change the profile points in legacy/default mode.

### Phase 2 — reference/working pair geometry and profile shift

- Implement the external and internal branches for `x1/x2`, addendum/dedendum,
  tooth thickness, working pressure angle, working radii, and centre distance.
- Implement the ISO signed internal adapter and verify it against the positive
  physical-radius branch.
- Replace the current contact-ratio inputs with working quantities while
  preserving the default values bit-for-bit within test tolerances.
- Update validation for invalid inverse-involute domains, pointed tips,
  internal tip/base clearance, root wall, and shift/centre-distance conflicts.

### Phase 3 — consumer migration

- Migrate mesh and SOLIDWORKS mates to the working distance.
- Migrate preview/report rows to explicit reference/working labels.
- Keep `pitch_r` in old paths until all consumers are migrated, then add
  deprecation warnings only if they will not spam normal CLI/GUI use.
- Preserve DXF/CSV topology and existing filename behavior for default mode.

### Phase 4 — generated root mode

- Implement the external rack envelope in pure math with named generated-root
  segments.
- Add an independent root-form validator and generated-root report values.
- Implement the internal cutter/shaper path separately; do not reuse the
  external trochoid with a sign flip unless an independent derivation proves it.
- Update the SOLIDWORKS section builder only after the pure geometry and preview
  tests establish the loop's orientation, closure, and material side.

### Phase 5 — GUI, CLI, JSON, and reports

- Add optional input fields and flags, including an explicit display of whether
  the set is in legacy or rack-generated root mode.
- Add migration/round-trip tests for old and new presets.
- Add reference and working quantities to reports without removing old aliases
  prematurely.
- Add non-default filename suffixes only with a dedicated collision test.

### Phase 6 — verification and release gate

- Run the complete suite, the ISO-only formula suite, CLI snapshots, preview/DXF
  tests, and all available SOLIDWORKS smoke tests.
- Build at least one external straight, external helical, internal straight, and
  internal helical pair in SOLIDWORKS with default and non-zero shift cases.
- Check centre distance, hand, clocking, interference, tooth count, tip/rim
  radius, and articulation in the generated assemblies.
- Only then consider changing the default root mode from legacy, and treat that
  as a deliberate compatibility decision rather than an incidental refactor.

## 10. Testing strategy

Every formula test must calculate its expected result from literals and the
test's own helper equations. It must not call a production helper that contains
the same equation and then compare the result to itself.

### Independent closed-form tests

Add a dedicated `tests/test_spur_iso_geometry.py` (or equivalent) containing:

- `m_t` and `alpha_t` conversion tests for beta = 0 and several non-zero helix
  angles;
- external reference diameter, base diameter, tip/root diameter, and tooth
  thickness tests at `x1=x2=0` and at positive/negative individual shifts;
- external `alpha_wt` and `a_w` tests calculated with a local `inv` expression;
- internal tests with positive physical `Z` that independently use
  `q = Z-z1` and `X=x2-x1`, including cases where only the ring is shifted and
  only the pinion is shifted;
- an ISO signed-adapter test proving that signed internal diameters and centre
  distance map to the same physical positive radii and placement distance;
- an explicit-centre-distance inversion test: independently solve `alpha_wt`,
  recover implied shift sum/difference, and assert that inconsistent inputs are
  rejected;
- external and internal contact-ratio tests with the two sign branches written
  longhand in the test;
- overlap and total-contact-ratio tests, including the straight-gear zero case;
- tip/root direction tests for rings and nominal-versus-generated-root tests;
- regression snapshots proving the default anchor values remain unchanged.

### Profile and root tests

- Test reference tooth thickness, normal thickness, transverse thickness, and
  backlash in the test file's own equations.
- Test that a shifted external tooth gets the expected addendum and root change.
- Test that a positive internal ring shift has the opposite radial addendum
  effect and the correct `x2-x1` working-angle sign.
- Test legacy profile loops for closure, symmetry, orientation, cap overshoot,
  and point topology exactly as the current suite does.
- For generated roots, test the rack profile independently, verify the envelope
  stays on the material side, verify tangent/transition continuity, and test
  undercut cases against independently sampled reference geometry.
- For internal generated roots, use a separate independent fixture and include
  a deliberate wrong-sign test that must fail.

### Consumer and compatibility tests

- Keep all existing spur geometry, mesh, internal, validate, preview, GUI, and
  CLI tests; update only labels or expected semantics where a test currently
  asserts the ambiguous word 'pitch.'
- Add JSON load tests for files without the new fields, files with transitional
  aliases, and canonical round trips.
- Add CLI tests showing old invocations produce the old report and new flags
  reach params and geometry.
- Add GUI tests for blank advanced fields, zero defaults, non-zero shifts,
  explicit working distance, validation messages, and readout labels.
- Add preview/DXF/CSV regression tests for both root modes.
- Keep SOLIDWORKS tests skipped when COM is unavailable, but require smoke builds
  in the release environment for all four pair types.

## 11. Current baseline and unresolved authoritative references

The unmodified repository was run from the requested `master` commit. The
ordinary command initially encountered Windows permission errors while pytest
enumerated the global temporary directory. Re-running the same suite with a
dedicated workspace basetemp produced the usable baseline:

```text
1047 passed, 59 skipped, 1 warning in 28.95s
```

The warning is pytest's inability to write the existing `.pytest_cache` path;
it did not affect the test results. No source files were changed by the audit.

The following references/formulas remain to be verified against licensed
standards text before implementation:

1. The complete ISO 53:1998 numeric profile table and profile-selection rules.
   Public secondary references consistently identify the common ISO 53 Profile
   A values as `h_aP*=1.0`, `h_fP*=1.25`, `c_P*=0.25`, and
   `rho_fP*=0.38`, but the official ISO page does not expose the table.
2. The exact ISO 21771-1 tooth-thickness equations and the precise handling of
   normal/transverse backlash and tolerances. ISO 21771-1 points tooth-thickness
   and backlash measurement toward ISO 21771-2; the current repository's
   symmetric `backlash/2` policy is an implementation compatibility choice.
3. The complete generated-root/trochoid equations and transition conditions in
   ISO 21771-1 Clauses 10/11 and Annex B, especially for an internal gear made
   with a pinion-type cutter. The plan names the required behavior but does not
   fabricate those equations.
4. The exact active-root/start-of-profile treatment to use in contact-ratio
   calculation once undercut/generated-root geometry is present. The nominal
   tip-circle formula is clear and independently testable; active/form limits
   require the licensed text and a deliberate implementation decision.

Until those points are verified, the implementation should call the new mode
'ISO-aligned' or 'ISO-derived', not 'ISO certified'.
