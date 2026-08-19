"""Where the two members sit, and how fast they turn. Pure math - no COM.

Shared by every gear type. What lives here is what does not depend on how the
axes are arranged: the clocking, the speed ratio, the little matrix helpers, and
the packing of a placement into the 16 doubles SOLIDWORKS wants. Each type's own
`mesh` module supplies the arrangement itself - a rotation to the shaft angle
for a bevel pair, a translation along the line of centres for a spur pair.

Coordinate system, matching the geometry modules: each part is built with its
own axis along +Z and a tooth **space** centred on the part's +X direction.
"""

from __future__ import annotations

import math

Matrix3 = tuple[tuple[float, float, float], ...]


def gear_clocking(z: int) -> float:
    """Rotation about the gear's own axis that puts a tooth at the contact line.

    The pinion presents a tooth *space* at the contact line - its space is
    centred on +X, and the contact line sits at angle 0 in the pinion's frame.
    To mesh, the gear must present a tooth *centre* there.

    **The contact line sits at pi in the gear's own frame**, which is what makes
    one formula serve both gear types, by two different routes:

    * *bevel*: the gear is rotated about Y by the shaft angle S, so its own +X
      maps to (cos S, 0, -sin S). Decomposing the contact line u against that
      gives u = cos(d2) * ag - sin(d2) * (cos S, 0, -sin S) - a negative
      component along the gear's +X, hence pi.
    * *spur*: the gear is not rotated at all, only translated to (a, 0, 0). The
      contact line runs back toward the pinion at the origin, i.e. along -X in
      the gear's frame, which is pi again.

    So: the gear's tooth centres sit at (k + 1/2) * tau, and we want the k whose
    tooth centre is nearest pi, then rotate by the difference. That lands on 0
    for odd tooth counts and half a pitch for even ones, with no need to
    special-case the parity.

    Written as `(pi - tau/2) % tau` this is numerically fragile: for z = 21,
    pi - tau/2 is exactly 10*tau, and in floating point the modulo can return
    tau instead of 0 - a full pitch of error. Picking the index explicitly and
    snapping the residual avoids sitting on that boundary.
    """
    tau = 2.0 * math.pi / z
    nearest = round(math.pi / tau - 0.5)
    phi = math.pi - (nearest + 0.5) * tau
    if abs(phi) < 1e-9:
        return 0.0
    return phi % tau


def angular_velocity_ratio(z1: int, z2: int, internal: bool = False) -> float:
    """w_pinion / w_gear, with each member's axis taken pointing away from the apex.

    Rolling without slipping along the contact line means the two surface
    velocities agree at every point of it, so `w1*a1 - w2*a2` - the relative
    angular velocity - must lie along that line. With a1 = (0, 0, 1),
    a2 = (sin S, 0, cos S) and u = (sin d1, 0, cos d1):

        x:   -w2 sin S      = k sin d1
        z:   w1 - w2 cos S  = k cos d1

    Eliminating k gives `w1 = w2 (cos S - sin S cot d1)`, and substituting the
    pitch angle - `cot d1 = (z2/z1 + cos S) / sin S`, which is just the pitch
    cone relation rearranged - collapses the whole thing to

        w1 / w2 = -z2 / z1

    for **any** shaft angle. Every S term cancels. The magnitude is the obvious
    one; the sign is the part worth having, and it says the two members always
    turn in *opposite* senses about their own outward axes - as true of a 45
    degree pair as of a right-angle one, and true of an external spur pair,
    which is this same derivation at S = 0 with the cones degenerated into
    cylinders.

    **An internal pair turns the same way**, so the sign is positive. The
    derivation above assumes the two pitch surfaces touch with their centres on
    opposite sides of the contact line, which is what "external" means. Roll a
    cylinder inside another instead and both centres are on the same side, the
    relative angular velocity reverses, and the ring follows the pinion rather
    than opposing it. That single sign is what makes a planetary train's output
    go the way it does.

    The tooth counts are the exact ratio here, not an approximation of one: the
    pitch surfaces are defined by them.
    """
    return (1.0 if internal else -1.0) * float(z2) / float(z1)


def gear_mate_ratio(z1: int, z2: int) -> tuple[float, float]:
    """The two numbers a SOLIDWORKS gear mate wants, pinion entity selected first.

    A gear mate holds `w1 * r1 = w2 * r2` for the values r1 and r2 given against
    its first and second selections, so feeding the tooth counts straight in
    gives `w1 / w2 = z2 / z1` - the magnitude from `angular_velocity_ratio`.

    The sign cannot be expressed in the ratio; it is the mate's Reverse flag,
    and it is the one bit of the placement that has to be measured rather than
    derived. For the bevel pair, Reverse off is correct - see the module
    docstring of `sw.bevel_assembly`. The spur pair is a *separate* measurement
    and does not inherit that answer.
    """
    return float(z1), float(z2)


def rot_y(a: float) -> Matrix3:
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def rot_z(a: float) -> Matrix3:
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def matmul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def apply(m: Matrix3, v) -> tuple[float, float, float]:
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def pinion_placement() -> Matrix3:
    """The pinion keeps the frame it was built in."""
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def to_array_data(
    m: Matrix3,
    translation=(0.0, 0.0, 0.0),
    scale: float = 1.0,
) -> list[float]:
    """Pack a rotation into the 16 doubles `IMathUtility.CreateTransform` wants.

    The rotation is stored **column-major** - the first three entries are the
    image of the X axis, and so on. Verified against SOLIDWORKS by building
    geometry from a known sketch transform and measuring where it landed;
    reading it row-major transposes the rotation and mirrors the part.

    Translation is in metres, matching the rest of the API. A bevel pair shares
    an apex and so always passes zero; a spur pair is the first caller that
    needs it, for the centre distance.
    """
    return [
        m[0][0], m[1][0], m[2][0],
        m[0][1], m[1][1], m[2][1],
        m[0][2], m[1][2], m[2][2],
        translation[0], translation[1], translation[2],
        scale,
        0.0, 0.0, 0.0,
    ]


def axis_of(m: Matrix3) -> tuple[float, float, float]:
    """Where the part's own +Z axis ends up. Handy for checking a placement."""
    return apply(m, (0.0, 0.0, 1.0))


def angle_between(u, v) -> float:
    dot = sum(a * b for a, b in zip(u, v))
    nu = math.sqrt(sum(a * a for a in u))
    nv = math.sqrt(sum(a * a for a in v))
    return math.acos(max(-1.0, min(1.0, dot / (nu * nv))))
