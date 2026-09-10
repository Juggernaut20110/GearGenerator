# ISO 21771 spur/cylindrical gear upgrade plan

Status: implementation plan retained as the design record; the parameter,
profile-shift, consumer, validation, root-mode, and independent-verification
phases described here are implemented in the current tree. The final review
does not treat this document as evidence of blanket standards compliance.

Implementation baseline: current branch after the commits documented by
`git log`; the independent release-gate checks are in
`tests/test_spur_iso21771.py` and `docs/iso21771_verification.md`.

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
21771-1:2024 also uses a signed negative-tooth-count convention for internal
gears. This repository deliberately preserves positive tooth counts plus an
`internal` flag and implements the equivalent physical positive-radius branch;
it does not expose a signed ISO adapter as a public API.

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
the paid standard text. The implementation therefore uses precise language:
the reference/working relationships and profile-shift branches are verified
against the equations recorded below, while the repository's backlash policy,
sampling, legacy root approximation, external rack envelope, and conservative
internal interference rule remain implementation choices or limitations.

## 2. ISO terminology mapped to this repository

The ISO symbol `z` is signed for internal gears in ISO 21771-1. In the table,
`Z` means the repository's positive magnitude (`p.z2` for a ring).

| ISO concept | Current implementation | Meaning |
|---|---|---|
| `m_n` normal module | `SpurSetParams.module`, `p.module` | Keep as the user-facing module and the rack/tool scale. |
| `m_t` transverse module | `p.transverse_module`; `SpurSetGeometry.transverse_module` | Keep as `m_n / cos(beta)` and expose it as a named derived quantity. |
| `z` tooth count | `p.z1`, `p.z2`, both positive | Keep the API positive. Convert to `z_iso = -Z` only inside ISO-style internal equations. |
| `alpha_n` normal pressure angle | `p.alpha_n`; input `pressure_angle` is degrees | Keep. This is the rack/tool angle for the helical case. |
| `alpha_t` transverse pressure angle | `p.alpha_t`; `geo.transverse_pressure_angle` | Keep as `atan(tan(alpha_n) / cos(beta))`. |
| `beta` helix angle | `p.beta` is signed by `hand`; each member has signed `beta` | Keep the signed member convention. Document that the pair's hand rule is a mesh/placement choice, not an ISO profile-shift quantity. |
| `d` reference diameter | `member.reference_d` | `pitch_r` remains a compatibility alias for `reference_r`; new code uses the explicit name. |
| `d_b` base diameter | `2 * member.base_r` | Keep, but derive from the reference circle and `alpha_t`, never from a working circle. |
| `d_a` tip diameter | `member.tip_d` | Keep as the nominal physical tip diameter, with the internal ring's tip being the smaller tooth radius. No signed ISO adapter is exposed. |
| `d_f` root diameter | `2 * member.root_r` | Keep as nominal root diameter. Do not use it to mean a generated root diameter. |
| `d_fE` / generated root diameter | `member.generated_root_d` in external straight rack mode | Available only for the opt-in external straight `rack_generated` mode; it is separate from nominal `root_d`. |
| `d_Ff` / root form diameter | `member.root_form_d` / `member.start_of_involute_d` | The solved start-of-involute/root-form diameter; separate from both nominal `d_f` and generated `d_fE`. |
| start of involute | `member.start_of_involute_r`, `member.start_of_involute_angle`, `member.involute_roll_parameter` | Available for the verified external straight rack-generated path. |
| undercut condition | `member.undercut` | Available for the verified external straight rack-generated path; internal and helical generated roots remain out of scope. |
| profile shift coefficient `x` | `profile_shift_1`, `profile_shift_2` | Dimensionless normal-module coefficients, both defaulting to zero. Transitional JSON spellings are migrated. |
| `k` tip alteration coefficient | `geo.tip_alteration_coefficient` | One pair-level coefficient from Clause 5.3.9 Eq. (76), or an explicit user override; never another `x_i`. |
| `h_w` working depth | `geo.working_depth` | Calculated from the actual operating centre distance and altered tip diameters using Clause 5.3.7 Eq. (73). |
| `c_1`, `c_2` tip clearance | `geo.tip_clearance_1`, `geo.tip_clearance_2` | Separate pinion-tip and gear/ring-tip clearances; `minimum_tip_clearance` is their minimum. |
| `h_aP*` | `basic_rack_addendum_factor` | User-editable data-model field, default `1.0`. |
| `h_fP*` | `basic_rack_dedendum_factor` property | Derived as `h_aP* + c_P*`; default `1.25`. |
| `c_P*` | `basic_rack_clearance_factor` | Explicit field, default `0.25`, used to derive the rack dedendum. |
| `rho_fP*` | `basic_rack_root_radius_factor` | Explicit field, default `0.38`; used by external straight rack-generated mode only. It is distinct from legacy `fillet_factor`. |
| `s` / tooth thickness | `reference_tooth_thickness`, `working_tooth_thickness` | Derived per-member transverse thicknesses at the reference and working circles; backlash definition is explicit. |
| `s_n` normal tooth thickness | `normal_tooth_thickness`, `working_normal_tooth_thickness` | Derived reference/working quantities, with normal/transverse conversion explicit. |
| `s_t` transverse tooth thickness | `reference_tooth_thickness`, `working_tooth_thickness` | Derived quantities including profile shift and the selected backlash allowance. |
| reference circle/cylinder | `member.reference_r` | Fixed by `m_t` and tooth count, not by working centre distance. |
| base circle/cylinder | `member.base_r` | Keep as the involute base circle. |
| tip circle/cylinder | `member.tip_r` | Keep as the tooth-end circle, with directional labels for an internal gear. |
| root circle/cylinder | `member.root_r` | Keep as the nominal root circle; add generated/form-root data separately. |
| working pitch circle/cylinder | `member.working_r`, `member.working_d` | Derived from the base radii and pair-level `alpha_wt`. |
| working pressure angle `alpha_wt` | `geo.working_pressure_angle` | Pair-level operating quantity; it equals `alpha_t` for zero shift/reference distance. |
| reference centre distance | `p.reference_centre_distance`, `geo.reference_centre_distance` | Derived from reference diameters; `p.centre_distance` remains the parameter compatibility alias. |
| working centre distance `a_w` | `geo.working_centre_distance` | Assembly/mesh distance; `geo.centre_distance` remains its compatibility alias. |
| profile shift | `p.profile_shift_combination` | External uses `x1 + x2`; internal physical positive-radius branch uses `x2 - x1`. No signed values are exposed. |
| centre-distance modification | `geo.centre_distance_modification` | Derived as `(a_w - a) / m_n`. |
| transverse contact ratio `epsilon_alpha` | `geo.transverse_contact_ratio` | Uses the actual `g_alpha` path when exact active-form limits are available; legacy, internal, and helical roots report an explicitly `approximate` nominal/full-involute basis. |
| path of contact `g_alpha` | `geo.path_of_contact` / `geo.actual_path_of_contact` | Actual line-of-action length between the resolved active limits; the report also exposes `contact_ratio_basis`. |
| tip-form diameter `d_Fa` | `member.tip_form_d` | Tip-form limit, separate from nominal `d_a`; currently equal to `d_a` because no separate tip corner/chamfer model is present. |
| start active profile `d_Nf` | `member.start_active_profile_d` | Pair-dependent active root/start limit selected from the member form and its mate's opposing limit. |
| active tip diameter `d_Na` | `member.active_tip_d` | Pair-dependent active tip limit selected from the member tip form and its mate's opposing root/form limit. |
| overlap ratio `epsilon_beta` | `geo.overlap_ratio` | ISO-named property alias for `axial_contact_ratio`. |
| total contact ratio `epsilon_gamma` | `geo.total_contact_ratio` | Sum of transverse and overlap ratios; report uses the ISO symbol. |

## 3. Current implementation audit

### Parameters and conversions

`gears/spur/params.py` is a frozen, JSON-serializable dataclass. The user-facing
inputs include module, tooth counts, face width, bore, hub, normal pressure
angle, helix angle, hand, internal arrangement, backlash, ring rim thickness,
and the two profile-shift coefficients `profile_shift_1` and
`profile_shift_2`. The basic-rack coefficients are also represented, although
the ordinary GUI exposes only the profile shifts. The important conversions are:

- `transverse_module = module / cos(beta)`;
- `alpha_t = atan2(tan(alpha_n), cos(beta))`;
- `reference_centre_distance = m_t * (z1 + z2) / 2` externally and
  `m_t * (z2 - z1) / 2` internally;
- `profile_shift_combination = x1 + x2` externally and `x2 - x1` internally;
- `alpha_wt` and `working_centre_distance` are solved from that combination,
  unless an explicit working distance is supplied.
- `tip_alteration_mode="iso_clearance"` resolves one pair-level `k` from
  Clause 5.3.9 Eq. (76); `legacy` means `k=0`, and `explicit` uses the
  supplied coefficient. Old JSON without the mode is migrated to `legacy`.

The normal/transverse conversions are used for a parallel-axis helical pair.
`alpha_t` is the transverse reference pressure angle, not the working pressure
angle. `p.centre_distance` is retained as a parameter-level alias for the
reference distance, while `geo.centre_distance` is retained as a geometry-level
alias for the working distance; new consumers use the explicit names.

`JsonParams` filters unknown JSON keys and allows dataclass defaults to fill
fields absent from old files. New fields are additive, and the transitional
spur names are handled by `SpurSetParams._migrate_json_data` rather than by
changing `JsonParams` globally.

### Geometry and involute profile

`gears/spur/geometry.py` contains the derived model in `compute_set`. The current
path is:

1. `reference_r = m_t*z/2` and `base_r = reference_r*cos(alpha_t)`.
2. Addendum and dedendum use `m_n` and the selected rack coefficients, with the
   external and internal radial signs handled in separate branches. The
   pair-level `k` is added to both physical addenda after profile shift and
   does not change reference/base/working circles.
3. The reference tooth thickness is formed in the normal system from `x_i`,
   projected to the transverse plane, and then reduced by half the deliberate
   backlash on each member.
4. `psi0` is calculated from that member's actual reference tooth thickness
   for an external gear, or its complementary space width for a ring.
5. Working radii come from the fixed base radii and `alpha_wt`. Active
   profile resolution then uses each member's analytical form limit and its
   mate's opposing limit on the line of action. The exact branch computes
   `d_Na`, `d_Nf`, and `g_alpha`; unsupported roots retain the nominal path and
   report an approximate basis.

The reference/working/profile-shift equations are independently checked in
`tests/test_spur_iso21771.py`. Non-standard rack values are represented and
validated for finite/ordered geometry, but the generated-root envelope is only
enabled for the external straight case described below.

`gears/involute.py` correctly isolates the pure involute and has a deliberate
external/internal split. In the default `legacy` mode, `flank_points` uses a
radial segment below the base circle and `root_fillet` fits a final-gear arc
based on `fillet_factor`; this is an approximation, not a claim that the
standard root is a circular arc of the final gear. In external straight
`rack_generated` mode, the below-base portion is instead the analytical rolling
envelope of the rounded basic-rack corner, sampled for CAD. Internal and
helical sections deliberately retain the legacy root path.

The current helical twist is `face_width*tan(beta)/reference_r`. The opposite-hand
external rule and same-hand internal rule are exercised by tests and are retained.
The section-count/sagitta logic, cap overshoot, and guide curve are
SOLIDWORKS implementation choices and are independent of profile shift.

### Contact and validation

`transverse_contact_ratio` uses the working pressure angle and working distance
to resolve the line-of-action limits. For the exact external
`rack_generated` branch, the active path is resolved from the analytical
member form limits. Unsupported roots retain the nominal path and report an
approximate basis.

```text
 external: sqrt(ra1^2-rb1^2) + sqrt(ra2^2-rb2^2) - a_w*sin(alpha_wt)
 internal: sqrt(ra1^2-rb1^2) - sqrt(ra2^2-rb2^2) + a_w*sin(alpha_wt)
```

These are the documented fallback equations for unsupported roots, divided by
the reference transverse base pitch `pi*m_t*cos(alpha_t)`. The internal signs
are correct for the repository's positive-radius/positive-distance convention
and are independently checked. The exact external rack-generated branch uses
the active-limit equations in the Contact ratios section below.

`validate.py` checks the derived reference/working geometry, tooth thickness,
lands, loop topology, contact ratios, blank wall, and profile-dependent
external undercut. It also checks working depth, both ISO tip clearances,
non-positive addendum, altered tip/form ordering, and pointed tips. Negative
tip clearance is an error. Internal running-pair interference now uses the
Clause 5.5.8.2 `CA < CT1` and `d_Nf2 < d_Ff2` conditions when the required
active/form geometry is available, plus the Clause 5.5.8.3 tip-to-tip rotation
criterion. The historical ten-tooth difference is retained only as a clearly
labelled non-normative warning.

### Mesh, preview, reports, and builders

- `gears/spur/mesh.py` translates by `geo.working_centre_distance`; reference
  circles remain the phase/reference geometry. Clocking and gear velocity ratio
  do not depend on profile shift.
- `gears/spur/preview.py` draws separate reference and working pitch circles and
  labels the readout with explicit reference/working terminology. All profile
  points come from `tooth_space_section`.
- `gears/spur/report.py` separates INPUT, REFERENCE GEOMETRY,
  WORKING/OPERATING GEOMETRY, and MEMBER GEOMETRY, including ISO symbols.
- `gears/sw/spur_part.py` revolves a blank to the current external tip or ring
  rim, loft-cuts the sampled tooth space, and patterns it. Its blank dimension
  plan relies on the outline's point count and H/V segment order. Shifted tip
  and root values must flow through `blank_outline`, not be recomputed in the
  COM layer.
- `gears/sw/spur_assembly.py` mates the axes at the working distance. Gear
  clocking remains a reference-tooth phase operation, not a working-circle
  substitution.
- `tools/build_spur.py`, `tools/build_spur_set.py`, and the terminal report
  accept additive `--x1`, `--x2`, `--tip-alteration-mode`, and explicit `k`
  flags. Existing saved JSON without the new mode is migrated to legacy.
- `gears/gui.py` exposes the automatic/legacy/explicit tip-alteration choice
  and optional explicit `k`, alongside the profile-shift inputs and derived
  readout.

## 4. Implemented parameter model

The parameter model separates user intent from derived geometry. The canonical
public fields are:

```python
profile_shift_1: float = 0.0
profile_shift_2: float = 0.0
basic_rack_addendum_factor: float = 1.0
basic_rack_clearance_factor: float = 0.25
basic_rack_root_radius_factor: float = 0.38
working_centre_distance: float | None = None
tip_alteration_mode: str = "iso_clearance"
tip_alteration_coefficient: float | None = None
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
generated mode the rack root-radius coefficient controls the rack corner and
`fillet_factor` is not used for the selected external root transition. It is
not silently reinterpreted as a basic-rack coefficient.

`working_centre_distance=None` means “derive the working distance from the two
profile shifts.” If a user supplies it, it becomes the assembly distance and
`alpha_wt` is solved from it. The validator compares the implied shift sum or
difference to the supplied `profile_shift_1`/`profile_shift_2` values and
reports a conflict rather than silently treating contradictory inputs as a
conjugate pair.

Defaults and migration rules implemented:

- all new numeric fields default to the current standard values or zero;
- `working_centre_distance=None` derives the old nominal distance at `x1=x2=0`;
- `root_geometry="legacy"` preserves the current section topology and point
  behavior;
- old JSON loads through dataclass defaults;
- transitional names `x1`, `x2`, `profile_shift1`, and `profile_shift2` are
  handled explicitly in `_migrate_json_data`, and canonical JSON writes only
  the new names;
- old CLI invocations do not acquire new required flags or new filename
  components.

## 5. Reference versus working geometry

The most important compatibility rule is: **`pitch_r` must never be silently
repurposed to mean a working radius.**

For each member, the derived object has:

```text
reference_r / reference_d     reference circle, d = m_t*|z|
working_r / working_d        operating circle, d_w = d_b/cos(alpha_wt)
base_r / base_d               involute base circle, d_b = d*cos(alpha_t)
tip_r / tip_d                 nominal tooth-end circle, directional for a ring
root_r / root_d                nominal root circle, directional for a ring
generated_root_r / generated_root_d  generated root boundary (`d_fE`) when available
root_form_r / root_form_d      root form / start-of-involute radius (`d_Ff`)
start_of_involute_angle        polar angle of the solved SOI
involute_roll_parameter        nominal involute roll at the solved SOI
undercut                       Clause 10.2 result when generated geometry applies
```

The compatibility property `pitch_r` returns `reference_r` and is documented as
such. Existing callers that mean “reference pitch circle” continue to get the
same value; new code uses `reference_r` or `working_r` by name. There is no
deprecation warning because ordinary legacy calls and the compatibility tests
still use the alias.

At the set level:

```text
reference_centre_distance = reference_r2 + reference_r1       external
reference_centre_distance = reference_r2 - reference_r1       internal
working_centre_distance   = working_r2 + working_r1             external
working_centre_distance   = working_r2 - working_r1             internal
centre_distance            compatibility alias of working_centre_distance
```

The physical radii above are positive. The implementation does not expose an
ISO signed internal adapter; instead it keeps the repository's positive ring
count/radius convention and applies the verified internal sign relationships
explicitly in `compute_set`. No mesh or SOLIDWORKS caller receives signed ISO
display values.

The tip/root naming used by the current code is useful for geometry but can be
misread in a ring. The physical names are retained and reports/readouts identify
“tip radius (inner)” and “root radius (outer)” for the ring. `outside_dia` remains
the diameter occupied by the teeth, not the outer rim diameter.

## 6. Profile-shift equations and sign audit

The equations below are the positive-radius implementation equations.
They intentionally use `Z = abs(z2)` for a ring and branch explicitly for
external/internal pairs. The equivalent ISO signed mapping is a derivation
boundary documented for review; it is not exposed as a production API.

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

### Reference and working tooth thickness

ISO 21771-1:2024 §5.6 distinguishes circumferential backlash at the reference
circle (`j_t`) from circumferential backlash at the working pitch circle
(`j_wt`), and also distinguishes transverse, normal, radial, and angular
forms. ISO 21771-2:2025 Clause 13 covers the corresponding parallel-axis
calculation cases, including §13.3, §13.4, and §13.7.1--§13.7.6.

The ideal external reference thickness in a transverse section is:

```text
s_n,i = m_n * (pi/2 + 2*x_i*tan(alpha_n))
s_t,i = s_n,i / cos(beta)
      = m_t * (pi/2 + 2*x_i*tan(alpha_n))
```

For the positive-radius internal ring convention used by this repository, a
positive ring shift removes material from the inward tooth addendum and the
ring tooth thickness uses the opposite `x_2` sign:

```text
s_t,ring = m_t * (pi/2 - 2*x_2*tan(alpha_n))
```

This is required for equal internal shifts to preserve the nominal working
condition; it is not a mechanical reuse of the external member equation. The
ring involute is still generated from the complement of its tooth thickness,
so its reference space is `pi*m_t - s_t,ring`.

The JSON/API field `backlash` is now interpreted through one explicit
`backlash_mode`:

```text
legacy_reference         input is the historical reference-circle allowance;
                         subtract backlash/2 from each reference tooth width
working_circumferential  input is j_wt at the actual working pitch circles
normal                   input is normal-base j_bn
```

The legacy mode remains the default for old JSON and deterministic API
compatibility. It is not relabelled as working backlash. For the two
standards-oriented modes, the requested working circumferential allowance is
allocated between the two members and solved back to their reference-circle
thicknesses:

```text
Delta_s_wt,1 = allocation * j_wt
Delta_s_wt,2 = (1-allocation) * j_wt
Delta_s_t,i  = Delta_s_wt,i * r_i/r_wi
```

The default allocation is 0.5 only as an explicit implementation default; ISO
does not require an equal split. The resulting working tooth widths are then
evaluated independently from the involute angular law and the pair closes on
the actual working circular pitch:

```text
j_wt = p_wt - s_wt,1 - s_wt,2
```

For `normal`, the current implementation converts `j_bn` through the working
transverse pressure angle and base-cylinder helix angle before applying the
same solve. The report exposes both the requested definition and the derived
`j_t`, `j_wt`, `j_bt`, `j_bn`, `j_wn`, `j_r`, and per-member angular quantities.
The exact published ISO 21771-2 equation pages were not available in the
public preview used for this review; these conversion equations are therefore
documented as the repository's analytical geometry derivation from the ISO
definitions, not as a claim of complete ISO 21771-2 equation-by-equation
conformance. The internal ring space continues to be formed from the
complement of its transverse tooth thickness.

The base-circle half-angle constants then become, for an external member,

```text
psi0 = s_t,actual/(2*r_i) + inv(alpha_t)
```

and, for the internal ring,

```text
psi0 = (pi*m_t - s_t,actual)/(2*r_2) + inv(alpha_t)
```

The existing `psi0` construction is therefore retained, but it consumes the
actual member tooth/space thickness produced by the selected backlash mode;
the working backlash is not created by moving `psi0` after the fact.

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
internally. The solver rejects an invalid inverse-involute domain and a
working distance that makes `alpha_wt` non-physical.

Working radii and diameters are then:

```text
r_wi = r_bi/cos(alpha_wt)
d_wi = 2*r_wi
```

The internal working distance is `r_w2-r_w1`, not a sum. This is the independent
sign check that should be used in tests and in the mesh placement.

### Contact ratios

For the exact external `rack_generated` branch, define the non-negative
line-of-action offsets

```text
q(r) = sqrt(max(0, r^2 - r_b^2))
q_wi = q(r_wi)
q_Ffi = q(d_Ffi / 2)
q_Fai = q(d_Fai / 2)
```

and the own tip/root paths:

```text
tip_path_i  = q_Fai - q_wi
root_path_i = q_wi - q_Ffi

active_tip_path_1 = min(tip_path_1, root_path_2)
active_tip_path_2 = min(tip_path_2, root_path_1)
g_alpha = active_tip_path_1 + active_tip_path_2

p_bt = pi*m_t*cos(alpha_t)
epsilon_alpha = g_alpha / p_bt
```

This is the positive-radius implementation of ISO 21771-1:2024 Clause 5.5.2.2
Eqs. (79)-(85), Clause 5.5.2.3 Eqs. (86)-(92), and Clause 5.5.6.2 Eq. (94),
with the approach/recess components corresponding to Eqs. (96) and (97). The
external/internal sign pair is deliberately retained in the fallback and in
the internal branch. For an internal ring, the physical ring tip is inward,
so its tip offset is `q_w - q_Fa` and the internal length uses the equivalent
subtraction branch.

When no verified `d_Ff` exists, the implementation retains the historical
nominal/full-involute tip-circle path, does not invent a root-form diameter,
and marks the result `contact_ratio_basis="approximate"`. This covers the
legacy, helical, and internal root paths currently supported by the CAD code.
The denominator is the reference transverse base pitch for the parallel-axis
cases implemented here, consistent with Clause 5.5.9.1 Eq. (113). If unequal
or crossed working base pitches are added later, the corresponding ISO
path-of-contact pitch must replace this shortcut.

For the helical pair, retain the axial overlap equation:

```text
epsilon_beta = face_width * abs(sin(beta)) / (pi*m_n)
epsilon_gamma = epsilon_alpha + epsilon_beta
```

Expose these as `overlap_ratio` and `total_contact_ratio`, while keeping
`axial_contact_ratio` as a compatibility alias. The signed hand controls twist;
the overlap magnitude uses the helix magnitude.

## 7. Root geometry status

ISO 21771-1 distinguishes the nominal root circle from generated/form geometry
and includes generated-root material in its later geometry sections and Annex B.
ISO 53 defines a basic rack tooth whose root transition is a circular fillet of
radius `rho_fP`; that does not mean the final gear root is the same circle. The
final root is the envelope left by the tool motion.

The current model provides both paths:

- `root_geometry="legacy"` uses the radial below-base segment and the
  `fillet_factor` final-gear circular approximation;
- external straight `root_geometry="rack_generated"` uses the analytical
  rolling envelope of the rounded basic-rack corner and named generated-root
  segments;
- internal and helical members remain on the legacy path, because their
  cutter-generated root equations were not independently verified.

### B: rack/cutter envelope (implemented limited mode)

The implemented external limited mode has a pure-math profile generator that:

1. builds the selected basic rack/tool profile in a normal section from
   `alpha_n`, `h_aP*`, `h_fP*`, `c_P*`, and `rho_fP*`;
2. rolls the rack/cutter against the gear and retains the analytical envelope
   in the transverse plane;
3. applies the Clause 10.2 undercut condition;
4. uses the Clause 10.3 no-undercut transition or solves the Eq. (275)/(276)
   trochoid/nominal-involute intersection when undercut exists;
5. samples the solved curve only for CAD, and supports the external rack/hob
   case only.

This gives the external straight model a generated root and a separately
reported start of involute/root-form diameter, and makes its undercut warning
meaningful, so it remains a separately selectable mode. It does not
claim an internal cutter/shaper envelope or a helical normal-to-transverse
projection of the rack corner.

### A: documented approximation (compatibility mode)

The existing radial-below-base and final-gear fillet path is retained and
labelled explicitly as `legacy` in reports and documentation. It remains useful
for preview/SOLIDWORKS compatibility and for old JSON. It is not described as
ISO-generated geometry, and validators warn when the real generated gear would
undercut.

The ISO standard does not, by itself, require this repository to use a specific
trochoidal construction for every CAD preview. The standard defines the
geometrical concepts and gives generated-form calculations; the decision to
implement the tool envelope, a simplified root, or a CAD-native sweep is an
implementation choice. The opt-in external straight envelope is the closest
implemented match to the generated-root goal; its limits are recorded above.

## 8. Compatibility strategy

### Geometry API (implemented)

- The model has explicit `reference_r`, `working_r`, `reference_d`, and
  `working_d`.
- Preserve `pitch_r` as a read-only alias for `reference_r`.
- `SpurSetGeometry` has `reference_centre_distance` and
  `working_centre_distance`; `centre_distance` is an alias of the latter.
- Keep `tip_r`, `root_r`, `base_r`, `addendum`, `dedendum`, `psi0`, and `twist`
  available during migration. Clarify in docstrings whether a quantity is
  nominal, generated, reference, or working.
- Keep `outside_dia` and `rim_radius` physical and positive. Never feed ISO
  signed ring diameters into SOLIDWORKS or preview transforms.

### Mesh, preview, and DXF/CSV (implemented)

- Mesh translation and the assembly distance mate use `working_centre_distance`.
- Reference-circle overlays use `reference_r` and include a separate
  `working_r` overlay.
- Preview titles/readouts say “reference” and “working” explicitly. The
  `pitch_r` compatibility property remains available to older callers.
- Tooth-space and CSV point topology stays unchanged in legacy mode. Generated
  root mode may add named `trochoid`/`generated_root` segments, but the existing
  named segments (`flank`, `riser`, `cap`, `root`) remain available to consumers.
- DXF export continues to export the selected scene, not a hidden signed ISO
  coordinate system.

### SOLIDWORKS (implemented consumer path)

- `blank_outline` remains the only source of blank dimensions. The builder keeps
  its current point count and alternating segment structure in legacy mode.
- The assembly uses the working distance, which equals the old distance at
  defaults. A shifted set therefore gets the correct operating placement.
- Part and assembly filenames remain unchanged for defaults and for shifted
  variants. A shifted build can therefore overwrite a standard build with the
  same module and tooth counts if the caller reuses the output directory/name.
- Expected radius checks use the physical tip/rim values and remain independent
  of ISO signed display values.
- The helix section-count, guide-curve, end-overshoot, hand, clocking, and
  pattern logic are retained unless a generated root adds more section segments.

### JSON, GUI, and CLI (implemented)

- `JsonParams`' unknown-key filtering is retained.
- Old JSON gets zero shifts, standard rack coefficients, derived nominal centre
  distance, and legacy root mode through dataclass defaults.
- New JSON writes canonical field names. Migration tests cover both an old file
  with no new keys and any transitional aliases.
- GUI exposes the two profile shifts as ordinary spur inputs. Rack coefficients,
  root mode, and an explicit working-distance override remain data-model/CLI
  options rather than ordinary GUI inputs.
- Both spur build tools and the report CLI accept additive shift flags without
  changing the meaning of existing flags. The old `--beta`, `--alpha`, `--hand`,
  `--backlash`, `--internal`, `--rim`, and sizing flags continue to produce the
  current default geometry.

## 9. Implementation phases and current status

### Phase 0 — audit and equation fixtures (complete)

- This plan and the source/equation checklist are retained as the design record.
- Independent expected-value fixtures cover external, helical, and internal
  anchor sets in `tests/test_spur_iso21771.py`.
- `pitch_r` and `centre_distance` are tested/documented as compatibility aliases
  for reference and working quantities respectively.

### Phase 1 — additive parameter model and compatibility aliases (complete)

- Dataclass fields, validation, JSON defaults, and migration hooks are present.
- Rack coefficients and explicit `legacy`/`rack_generated` root modes are present.
- Explicit `reference_*` and `working_*` names coexist with compatibility aliases.
- The default legacy profile retains the previous geometry.

### Phase 2 — reference/working pair geometry and profile shift (complete)

- External and internal branches for `x1/x2`, addendum/dedendum, tooth thickness,
  working pressure angle, working radii, and centre distance are implemented.
- The repository uses explicit positive-radius internal equations equivalent to
  the signed ISO convention; a signed adapter is not exposed.
- Contact ratios use working distance/angle with the reference transverse base
  pitch, and zero-shift defaults retain their previous values.
- Validation covers inverse-involute domains, pointed tips, internal tip/base
  clearance, root wall, and shift/centre-distance conflicts.

### Phase 3 — consumer migration (complete)

- Mesh and SOLIDWORKS mates use the working distance.
- Preview/report rows use explicit reference/working labels.
- `pitch_r` remains a documented compatibility alias without warning noise.
- DXF/CSV topology and default filename behavior are preserved.

### Phase 4 — generated root mode (limited implementation complete)

- The external straight rack envelope is implemented in pure math with named
  generated-root segments and independent loop/root tests.
- Generated-root values are reported separately from nominal root values.
- No internal cutter/shaper envelope or helical projected rack-root envelope is
  claimed; those paths remain legacy.

### Phase 5 — GUI, CLI, JSON, and reports (complete)

- The GUI exposes profile shifts, both spur build tools and the report accept
  `--x1`/`--x2`, JSON migration/round trips are tested, and reports expose
  reference/working quantities and root mode.
- Non-default filename suffixes were not added; callers must choose distinct
  output paths when building shifted variants.

### Phase 6 — verification and release gate (partially complete)

- The complete Python suite, independent ISO equation suite, CLI, preview/DXF,
  and compiled-geometry gate are run as release checks where their dependencies
  are available; results are recorded in the verification document.
- SOLIDWORKS smoke builds are environment-dependent and are not verified in
  this checkout without a COM/CAD session.
- The default root mode remains `legacy` as a deliberate compatibility choice.

## 10. Testing strategy

Every formula test must calculate its expected result from literals and the
test's own helper equations. It must not call a production helper that contains
the same equation and then compare the result to itself.

### Independent closed-form tests

`tests/test_spur_iso21771.py` contains:

- `m_t` and `alpha_t` conversion tests for beta = 0 and several non-zero helix
  angles;
- external reference diameter, base diameter, tip/root diameter, and tooth
  thickness tests at `x1=x2=0` and at positive/negative individual shifts;
- external `alpha_wt` and `a_w` tests calculated with a local `inv` expression;
- internal tests with positive physical `Z` that independently use
  `q = Z-z1` and `X=x2-x1`, including cases where only the ring is shifted and
  only the pinion is shifted;
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
- Internal generated roots are intentionally not implemented or tested; the
  external rack-root algorithm is not applied to rings.

### Consumer and compatibility tests

- Existing spur geometry, mesh, internal, validate, preview, GUI, and CLI tests
  cover compatibility and the explicit reference/working labels.
- JSON tests cover files without new fields, transitional aliases, and
  canonical round trips.
- CLI/GUI tests cover zero defaults and non-zero profile shifts.
- Preview/DXF/CSV tests cover legacy and external straight rack-generated modes;
  SOLIDWORKS tests remain skipped when COM is unavailable.

## 11. Current verification boundary and limitations

The independent equation and compatibility results are recorded in
`docs/iso21771_verification.md`; repository check results belong to the review
record rather than this historical plan. The following boundaries remain:

1. The default basic-rack coefficients are represented as the common Profile A
   values `h_aP*=1.0`, `h_fP*=1.25`, `c_P*=0.25`, and `rho_fP*=0.38`. They are
   implementation inputs; profile selection and manufacturing tolerances are
   not modelled.
2. The symmetric `backlash/2` policy is a compatibility choice for the
   user-facing circular reference-pitch backlash. It is not a full ISO 21771-2
   tolerance or inspection model.
3. The generated-root construction is verified only for the external straight
   rack-generated mode. Internal and helical roots use the legacy approximation;
   no internal cutter/shaper envelope is claimed. The Clause 10.4 trochoid
   curvature equations are recorded in the verification document but are not
   implemented as a reported result.
4. Active contact limits and the shortened path are exact only for the
   external straight rack-generated branch with both generated root forms
   available. Legacy, internal, and helical roots retain a nominal/full-
   involute approximation and are labelled as such. Internal running-pair
   interference is checked from working geometry, but the current internal
   legacy root does not provide an independently verified `d_Ff`, so the
   `d_Nf2 < d_Ff2` branch remains a documented warning rather than a fabricated
   nominal-`d_f` substitution. The check is transverse for helical pairs and
   does not cover internal cutter/shaper geometry or Clause 11 cutter
   interference.
5. Sampling density, loft interpolation, blank construction, SOLIDWORKS mating,
   manufacturing tolerances, and inspection definitions are not normative ISO
   claims and require environment-specific validation.

Accordingly, documentation and reports call the implementation ISO-aligned or
ISO-derived and do not claim ISO certification or blanket ISO 21771 compliance.
