#include "core/planetary/mesh.hpp"

#include "core/placement/placement.hpp"

#include <cmath>

namespace geargen::core::planetary::mesh {

double carrier_angle(const SetGeometry& geometry, int index) noexcept
{
    return kTau * index / geometry.parameters.n_planets;
}

double sun_clocking() noexcept
{
    return 0.0;
}

double planet_clocking(const SetGeometry& geometry, int index) noexcept
{
    const auto& p = geometry.parameters;
    return carrier_angle(geometry, index) *
               (p.z_sun + p.z_planet) / p.z_planet +
           placement::gear_clocking(p.z_planet);
}

double ring_clocking(const SetGeometry& geometry, int index) noexcept
{
    const auto& p = geometry.parameters;
    return carrier_angle(geometry, index) * (p.z_ring() + p.z_sun) /
               p.z_ring() +
           placement::gear_clocking(p.z_planet) * p.z_planet / p.z_ring() +
           kPi / p.z_ring();
}

Point3 planet_translation(const SetGeometry& geometry, int index) noexcept
{
    const double phi = carrier_angle(geometry, index);
    return {geometry.centre_distance_mm * std::cos(phi),
            geometry.centre_distance_mm * std::sin(phi), 0.0};
}

Matrix3 member_placement(double clocking_rad) noexcept
{
    return placement::rot_z(clocking_rad);
}

std::pair<double, double> sun_planet_ratio(const SetGeometry& geometry) noexcept
{
    return placement::gear_mate_ratio(geometry.parameters.z_sun,
                                      geometry.parameters.z_planet);
}

std::pair<double, double> planet_ring_ratio(const SetGeometry& geometry) noexcept
{
    return placement::gear_mate_ratio(geometry.parameters.z_planet,
                                      geometry.parameters.z_ring());
}

} // namespace geargen::core::planetary::mesh
