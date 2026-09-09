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

Mesh tests also verify the working-distance translation, external versus
internal phase target, and the pair-level backlash split.

## Equations and reference source

The equations follow the terminology and implementation equations recorded in
[`docs/iso21771_spur_plan.md`](iso21771_spur_plan.md), based on the concepts of
ISO 21771-1:2024 and the ISO 53:1998 basic rack. The independent tests write
the following directly:

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

s_ni          = m_n (pi/2 + 2 x_i tan(alpha_n))
s_ti          = s_ni / cos(beta) - backlash/2

epsilon_alpha = (B1 +/- B2 +/- a_w sin(alpha_wt)) /
                (pi m_t cos(alpha_t))
epsilon_beta  = face_width |sin(beta)| / (pi m_n)
epsilon_gamma = epsilon_alpha + epsilon_beta
```

The plus/minus branch in the contact length is external
`B1+B2-a_w sin(alpha_wt)` and internal
`B1-B2+a_w sin(alpha_wt)`, with
`B_i=sqrt(max(0,r_ai^2-r_bi^2))`. Ring addendum and dedendum use the explicit
positive-radius internal convention from the plan, so the ring tip is inward
and its root is outward.

Working tooth thickness is not a stored production field. Where checked, the
test derives it from the reported working radius and `psi0` using the
involute angular law, with the complementary ring-space expression for an
internal member.

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

The rack-generated root envelope is verified only for the existing opt-in
external straight construction. Exact cutter-generated root geometry for
helical and internal gears remains outside this implementation. The nominal
tip-circle contact-ratio formula is verified, but a complete active-profile
contact-ratio calculation for every generated undercut form is not claimed.

The tests verify backlash as the current reference-circle tooth-thinning policy
and verify the phase/working-distance algebra. They do not replace a physical
tooth-contact analysis, elastic backlash calculation, or a SOLIDWORKS
interference study. Sampled profiles are checked against their analytical
circles and involute transition, but CAD loft tolerances and cutter
manufacturability still require downstream validation.
