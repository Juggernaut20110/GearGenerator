#include "core/spur/mesh.hpp"

#include "core/placement/placement.hpp"

namespace geargen::core::spur::mesh {

Matrix3 gear_placement(double clocking_rad) noexcept
{
    return placement::rot_z(clocking_rad);
}

Point3 gear_translation(const SetGeometry& geometry) noexcept
{
    const double sign = geometry.parameters.internal ? -1.0 : 1.0;
    return {sign * geometry.working_centre_distance_mm, 0.0, 0.0};
}

double internal_gear_clocking(int teeth) noexcept
{
    return teeth == 0 ? 0.0 : kPi / teeth;
}

double clocking_for(const SetGeometry& geometry) noexcept
{
    return geometry.parameters.internal
               ? internal_gear_clocking(geometry.gear.teeth)
               : placement::gear_clocking(geometry.gear.teeth);
}

std::pair<double, double> mate_ratio(const SetGeometry& geometry) noexcept
{
    return placement::gear_mate_ratio(geometry.pinion.teeth,
                                      geometry.gear.teeth);
}

} // namespace geargen::core::spur::mesh
