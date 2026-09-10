# ISO 21771 spur-gear verification

This document records the independent verification pass for the cylindrical
involute implementation. The tests are in
[`tests/test_spur_iso21771.py`](../tests/test_spur_iso21771.py). They calculate
expected values from equations written in that test module rather than calling
the production conversion, working-geometry, or contact-ratio helpers.

## Cases covered

The independent matrix includes:

- external straight pairs at `x1=x2=0`, positive and negative individual
  shift, unequal shifts with zero total shift, and positive total shift;
- external helical pairs at `beta=15 deg` and `beta=30 deg`, with both
  balanced and positive total profile shift;
- internal straight pairs at zero shift, pinion shift, unequal zero-total
  shift, and positive total shift;
- an internal helical pair at `beta=15 deg` with nonzero individual shifts;
- dimensional scaling, member-swapping symmetry, constant external shift-sum,
  exact zero-shift compatibility, and the `beta -> 0` limit;
- hostile but geometrically usable cases near the external undercut boundary,
  large positive and negative shifts, the allowed 14.5 and 25 degree pressure
  angles, and small and relatively large helix angles;
- legacy external/internal/helical profiles and the opt-in external straight
  rack-generated root profile.

The rack-generated-root regression matrix additionally covers a standard
20-degree gear above the rounded-rack undercut limit (`z=30`), an undercut
case (`z=12`), the theoretical boundary neighborhood (`z=17` and `z=18`),
positive and negative profile shift, the old rack flank/tip tangency point,
and independent radial/angular SOI coincidence checks.

Mesh tests also verify the working-distance translation, external versus
internal phase target, and the pair-level backlash split.

`tests/test_spur_iso21771_2.py` independently recomputes working tooth widths
for unshifted and shifted external spur, external helical, internal spur, and
internal helical pairs. It verifies requested `j_wt`, explicit allocation,
normal-base conversion, internal tooth/space complement behavior, and the
distinction between legacy reference and working backlash.

## Equations and reference source

The equations follow the terminology and implementation equations recorded in
[`docs/iso21771_spur_plan.md`](iso21771_spur_plan.md), based on the concepts of
ISO 21771-1:2024 and the ISO 53:1998 basic rack. The independent tests write
the following directly for nominal/full-involute and unsupported-root cases;
the exact active-profile path equations are recorded below:

```text
m_t       = m_n / cos(beta)
alpha_t   = atan2(tan(alpha_n), cos(beta))
d_i       = m_t z_i
d_bi      = d_i cos(alpha_t)

external: a = m_t (z1 + z2) / 2,  X = x1 + x2
internal: a = m_t (z2 - z1) / 2,  X = x2 - x1

inv(alpha_wt) = inv(alpha_t) + 2 X tan(alpha_n) / q
a_w           = a cos(alpha_t) / cos(alpha_wt)
r_wi          = r_bi / cos(alpha_wt)

s_ni          = m_n (pi/2 + 2 x_i tan(alpha_n))       [external]
s_t2,ring     = m_t (pi/2 - 2 x_2 tan(alpha_n))        [internal convention]
s_ti,legacy   = s_ti - backlash/2                       [legacy_reference]

Delta_s_wt,1  = allocation * j_wt
Delta_s_wt,2  = (1-allocation) * j_wt
Delta_s_t,i   = Delta_s_wt,i * r_i/r_wi
s_wt           = p_wt - (member tooth/space complement)
j_wt           = p_wt - s_wt,1 - s_wt,2

epsilon_alpha = (B1 +/- B2 +/- a_w sin(alpha_wt)) /
                (pi m_t cos(alpha_t))
epsilon_beta  = face_width |sin(beta)| / (pi m_n)
epsilon_gamma = epsilon_alpha + epsilon_beta
```

### Backlash definitions and implementation boundary

The primary terminology source is ISO 21771-1:2024 §5.6:

- §5.6.2: transverse backlash `j_bt`;
- §5.6.3: circumferential backlash, including reference `j_t` and working
  circumferential `j_wt` quantities;
- §5.6.4: normal/base backlash `j_bn` and the working normal quantity `j_wn`;
- §5.6.5: radial backlash `j_r`;
- §5.6.6: angular backlash `phi_j`.

The published target for tooth-thickness/backlash calculation is
[ISO 21771-2:2025](https://www.iso.org/cms/%20render/live/en/sites/isoorg/contents/data/standard/07/83/78378.html),
Clause 13: §13.3 axis-related backlash, §13.4 non-datum
backlash where applicable, and §13.7.1--§13.7.6 for circumferential,
transverse, normal, axial, radial, and angular expressions. Clause 4.7.16 and
4.7.17 are the published contents entries for tooth-thickness and space-width
calculation. The publicly accessible [publisher sample]
(https://standards.iteh.ai/catalog/standards/iso/861da4f1-daa6-49da-9846-3274d5f3869e/iso-21771-2-2025)
was used to verify the edition and clause headings; it does not expose the
full normative equation pages.

The implementation evaluates actual involute tooth widths at `r_w` and sets
the member reference widths so their working-pitch closure produces the
requested definition. For `working_circumferential`, the independent identity
checked by the tests is:

```text
s_wt,external = 2*r_w*(psi0 - inv(alpha_wt))
s_wt,internal = 2*r_w*(pi/z - (psi0 - inv(alpha_wt)))
j_wt          = p_wt - s_wt,1 - s_wt,2
```

The verified ISO 21771-1 equations used for the derived conversions are
§5.6.2 Eq. (123), `j_bt = j_wt cos(alpha_wt)`; §5.6.3 Eq. (124),
`j_wt = j_bn/(cos(alpha_wt) cos(beta_b))`; §5.6.4 Eq. (126),
`j_bn = j_bt cos(beta_b)`; and §5.6.5 Eq. (127),
`j_r ~= j_wt/(2 tan(alpha_wt))`. The working normal value `j_wn` is
calculated from the working normal pitch and the two working normal tooth
thicknesses, giving `j_wn = j_wt cos(beta_w)` for this unmodified pair model.
Angular backlash follows §5.6.6 from the working circumferential arc and each
member's working pitch radius.

For `normal`, the input is explicitly `j_bn`; the implementation derives the
working circumferential target through the transverse and base-cylinder
projections and verifies the resulting `j_bn` independently. `j_t` remains a
reported resulting reference-circle quantity and can differ from the input in
shifted geometry because profile shift changes the natural reference tooth
width sum.

The published ISO 21771-2 formula pages were not present in the public sample
preview available for this review. Therefore the clause references above are
used for terminology and calculation scope, while the conversion equations
implemented here are documented as analytical derivations from those
definitions. This change does not claim full ISO 21771-2 conformance.

### Pair-level tip alteration and clearance

The pair-level addendum/clearance implementation uses ISO 21771-1:2024
Clause 4.6.4, Clause 4.6.5 Eq. (36), and Clauses 5.3.7–5.3.9. In particular:

- Clause 5.3.7 Eq. (73): `h_w = (d_a1 + d_a2)/2 - a_w`.
- Clause 5.3.8 Eqs. (74) and (75):
  `c_1 = a_w - d_a1/2 - d_fE2/2` and
  `c_2 = a_w - d_a2/2 - d_fE1/2` for the external signed-radius form.
- Clause 5.3.9 Eq. (76): for an external pair,
  `k = (a_w - a)/m_n - (x_1 + x_2)`; converting ISO's signed internal
  tooth-count convention to this repository's positive-radius convention gives
  `k = (a - a_w)/m_n + (x_2 - x_1)` for an internal pair.
- Clause 4.6.5 Eq. (36): each tip diameter includes `+2 k m_n`.

The implementation stores one pair-level `k`; it does not invent `k_1` and
`k_2`. The physical internal expressions are converted explicitly: the ring
tip radius decreases with positive `k`, while the pinion tip radius increases.
The two internal clearances are therefore retained separately rather than
reusing external signs. For non-generated legacy/helical/internal roots,
nominal `d_f` is used as a documented fallback because this repository does
not yet have the corresponding analytical `d_fE` construction.

`x_i` remains generating/profile shift: it changes tooth thickness, dedendum,
and the working pair condition. `k` is a later pair-level tip/addendum change:
it changes addendum, tip diameter, working depth, and tip clearance, but not
reference diameter, base diameter, reference tooth thickness, or working
pressure angle. Under this repository's external positive-radius convention,
negative `k` shortens the external tips and positive `k` lengthens them; the
internal ring direction is reversed by the physical-radius conversion. New
parameter sets default to `tip_alteration_mode=
"iso_clearance"`; JSON files that predate this field are migrated to the
explicit `legacy` (`k=0`) mode so old saved geometry is not silently changed.
`explicit` mode accepts a user-supplied coefficient.

The plus/minus branch in the contact length is external
`B1+B2-a_w sin(alpha_wt)` and internal
`B1-B2+a_w sin(alpha_wt)`, with
`B_i=sqrt(max(0,r_ai^2-r_bi^2))`. Ring addendum and dedendum use the explicit
positive-radius internal convention from the plan, so the ring tip is inward
and its root is outward.

Working tooth thickness is now exposed as a derived production quantity. The
independent tests still derive it from the reported working radius and `psi0`
using the involute angular law, with the complementary ring-space expression
for an internal member.

### Active profile, path of contact, and generated-root references

The primary source for this change is ISO 21771-1:2024, Clauses 9.6, 9.7 and
10.1–10.4. The publication record is
[ISO 21771-1:2024](https://www.iso.org/standard/84949.html); the
clause/equation text used for this review was checked against the
[publisher/sample preview of the same edition](https://previewnorm.com/iso/ISO%2021771-1-2024%20PDF.pdf).

The implementation uses these exact references:

- Clause 5.5.2.1 defines the active profile limits and distinguishes the
  root-form/start-of-involute diameter `d_Ff`, the tip-form diameter `d_Fa`,
  and the pair-dependent active limits `d_Nf` and `d_Na`.
- Clause 5.5.2.2 Eqs. (79)-(85) gives the pinion active root/tip limits and
  the corresponding line-of-action angles/roll parameters. Clause 5.5.2.3
  Eqs. (86)-(92) gives the corresponding gear limits. The implementation
  selects the smaller usable own/mating interval on the line of action and
  reconstructs the corresponding physical diameter from the base-circle
  offset. This is the positive-radius external equivalent of those equations;
  the internal sign convention is kept separate.
- Clause 5.5.4 defines the line of action and its operating limits A and E.
- Clause 5.5.5 Eq. (93) defines the form-overlap quantity `c_F`.
- Clause 5.5.6.2 Eq. (94) defines the path of contact `g_alpha = AE`.
  Eqs. (96) and (97) define the approach/recess components `g_a` and `g_f`.
  The implementation evaluates the same quantities from analytical
  base-circle offsets, using the external and internal positive-radius sign
  branches documented in the plan.
- Clause 5.5.9.1 Eq. (113) defines `epsilon_alpha = g_alpha / p_et`;
  Clause 5.5.9.3 defines the overlap ratio and Clause 5.5.9.5 the total
  contact ratio. For the parallel-axis cases implemented here, `p_et` is the
  transverse base pitch `p_bt`.

- Clause 10.1, Eqs. (264)–(272): polar involute/trochoid definitions and the
  rack tip parameter. The code retains an analytical rolling-envelope form in
  Cartesian coordinates; it is sampled only for CAD output.
- Clause 10.2, Eq. (273): the undercut inequality. For the external straight
  case, the inverted generating rack maps its tool tip depth to
  `m_n (h_fP* - x)` and its transverse tip radius to `m_n rho_fP*`, giving the
  implemented straight-gear equivalent
  `r_ref sin(alpha)^2 - m_n[h_fP* - x - rho_fP*(1 - sin(alpha))] < 0`.
- Clause 10.3, Eq. (274): the no-undercut root-form radius at the rack
  corner/flank transition. The independent tests reproduce this radius from
  the local rack geometry.
- Clause 10.3, Eqs. (275) and (276): undercut SOI is the physical intersection
  satisfying both `r_tro = r_inv` and `eta_tro = eta_inv`.
- Clause 10.3, Eq. (277): radial equality can eliminate the involute
  parameter as a function of the trochoid parameter. The solver does the
  equivalent elimination by calculating the involute roll parameter from the
  analytical trochoid radius, then bounded-solving the angular residual.
- Clause 10.4, Eqs. (280)–(286): radius of curvature of the trochoid. These
  equations are recorded as a source boundary but are not used to locate the
  SOI or exposed as a claimed curvature result in this change.

The active-profile implementation exposes `path_of_contact` (the actual
`g_alpha`) and `contact_ratio_basis` on `SpurSetGeometry`. Each member also
exposes `tip_form_d` (`d_Fa`), `start_active_profile_d` (`d_Nf`), and
`active_tip_d` (`d_Na`). In the exact branch, `contact_ratio_basis` is
`active_profile`: it is currently available only when both members are
external straight members using the analytical rack-generated root, so both
`d_Ff` values are known. The current tip model has no separate rounded or
chamfered tooth-end form, therefore `d_Fa` equals the member's nominal `d_a`
in that branch; it remains a distinct field so a future tip-form model cannot
collapse the ISO quantities.

For legacy roots, internal pairs, and helical pairs, the generated root or
cutter form is not verified. Those configurations retain the historical
nominal/full-involute tip path and report `contact_ratio_basis` as
`approximate`; the implementation does not manufacture a `d_Ff` value for
them. This is a declared approximation, not an ISO active-profile result.

The distinction required by Clauses 9.6 and 9.7 is preserved: `d_fE` is the
generated root-circle boundary produced by the selected rack envelope, `d_f`
is the nominal root diameter, and `d_Ff` is the root-form/start-of-involute
diameter returned by the solved transition. They are not aliases.

The pair-level tests independently verify `k`, `h_w`, `c_1`, `c_2`, `d_a1`,
and `d_a2` for unshifted, positive/negative total shift, unequal shift
distribution, helical, internal, and high-positive-shift cases. The high-shift
case is also run in legacy mode to prove that negative clearance is reported
as an error rather than returned as overlapping geometry.

## Tolerances

Closed-form dimensions, angles, and ratios are checked to `1e-11` in the test
module, with `1e-10` for the derived working tooth thickness and `2e-8` radians
for sampled involute points. Circle samples use `1e-8 mm`. These tolerances are
for double-precision equation consistency, not manufacturing tolerances or
CAD accuracy claims.

The scaling test doubles all linear inputs that participate in the case,
including module, face width, bore, hub thickness, and backlash. It requires
linear derived quantities to double, angles and profile-shift coefficients to
remain unchanged, and all contact ratios to remain unchanged.

## Limitations and items not verified

This pass verifies the implemented analytical relationships; it does not claim
full ISO certification or conformity assessment. It does not verify every
manufacturing tolerance, datum, or inspection definition in ISO 21771-1:2024.

The rack-generated root envelope and SOI intersection are verified only for
the existing opt-in external straight construction. Exact cutter-generated
root geometry for helical and internal gears remains outside this
implementation. Clause 10.4 curvature is not implemented. Active-profile
path/contact-ratio limits are implemented only for the exact external straight
rack-generated branch; legacy, internal, and helical results remain explicitly
approximate as described above.

ISO 21771-1 Clause 5.3.9 describes Eq. (76) as an estimate for many gear sets
and notes that internal-pair addendum limits can prevent the calculated `k`
from being realized. This implementation validates the resulting pair
clearances, addendum, active form, and tip land, but does not claim a complete
internal tip-to-tip interference analysis or manufacturing feasibility proof.

The tests verify the legacy reference-circle, working-circumferential, and
normal-base input paths from analytical member widths, and verify the
phase/working-distance algebra. They do not replace a physical tooth-contact
analysis, elastic backlash calculation, or a SOLIDWORKS interference study.
Sampled profiles are checked against their analytical circles and involute
transition, but CAD loft tolerances and cutter manufacturability still require
downstream validation. The publicly available ISO 21771-2:2025 preview did not
expose its full equation pages, so the implemented conversion equations are
not claimed as a complete equation-by-equation ISO conformance implementation.
