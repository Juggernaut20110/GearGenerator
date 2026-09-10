#include "core/hypoid/mesh.hpp"

#include "core/placement/placement.hpp"

#include <cmath>

namespace geargen::core::hypoid::mesh {

Matrix3 pinion_placement(const SetGeometry& geo) noexcept
{
    const auto [theta, unused] = contact_azimuths(geo);
    (void)unused;
    return placement::rot_z(theta);
}

double gear_clocking(const SetGeometry& geo) noexcept
{
    const auto [unused, target] = contact_azimuths(geo);
    (void)unused;
    const double tau = kTau / geo.gear.z;
    const double nearest = std::round(target / tau - 0.5);
    const double residual = target - (nearest + 0.5) * tau;
    if (std::abs(residual) < 1e-9) return 0.0;
    const double wrapped = std::fmod(residual, tau);
    return wrapped < 0.0 ? wrapped + tau : wrapped;
}

Matrix3 gear_placement(double shaft_angle_rad, double clocking_rad) noexcept
{
    return placement::matmul(placement::rot_y(shaft_angle_rad),
                             placement::rot_z(clocking_rad));
}

Point3 contact_point(const SetGeometry& geo)
{
    const auto [theta, unused] = contact_azimuths(geo);
    (void)unused;
    const auto& m = geo.pinion;
    return {m.pitch_radius_mm * std::cos(theta),
            m.pitch_radius_mm * std::sin(theta),
            m.cone_distance_mm * std::cos(m.pitch_angle_rad)};
}

Point3 gear_translation(const SetGeometry& geo)
{
    const auto [theta1, theta2] = contact_azimuths(geo);
    const auto& a = geo.pinion;
    const auto& b = geo.gear;
    const Point3 pinion_point{a.pitch_radius_mm * std::cos(theta1),
                              a.pitch_radius_mm * std::sin(theta1),
                              a.cone_distance_mm * std::cos(a.pitch_angle_rad)};
    const Point3 local{b.pitch_radius_mm * std::cos(theta2),
                       b.pitch_radius_mm * std::sin(theta2),
                       b.cone_distance_mm * std::cos(b.pitch_angle_rad)};
    const Point3 rotated = placement::apply(placement::rot_y(geo.parameters.sigma()), local);
    return {pinion_point.x - rotated.x, pinion_point.y - rotated.y,
            pinion_point.z - rotated.z};
}

Point3 gear_contact_point(const SetGeometry& geo)
{
    const auto [unused, theta2] = contact_azimuths(geo);
    (void)unused;
    const auto& b = geo.gear;
    const Point3 local{b.pitch_radius_mm * std::cos(theta2),
                       b.pitch_radius_mm * std::sin(theta2),
                       b.cone_distance_mm * std::cos(b.pitch_angle_rad)};
    const Point3 rotated = placement::apply(placement::rot_y(geo.parameters.sigma()), local);
    const Point3 translation = gear_translation(geo);
    return rotated + translation;
}

double axis_offset(const SetGeometry& geo) noexcept
{
    return std::abs(geo.parameters.offset);
}

double skew_axis_distance(Point3 origin1, Point3 axis1, Point3 origin2,
                          Point3 axis2) noexcept
{
    const Point3 cross{axis1.y * axis2.z - axis1.z * axis2.y,
                       axis1.z * axis2.x - axis1.x * axis2.z,
                       axis1.x * axis2.y - axis1.y * axis2.x};
    const double magnitude = norm(cross);
    const Point3 delta = origin2 - origin1;
    if (magnitude < 1e-12) {
        const double projection = dot(delta, axis1);
        return norm(delta - axis1 * projection);
    }
    return std::abs(dot(delta, cross)) / magnitude;
}

} // namespace geargen::core::hypoid::mesh
