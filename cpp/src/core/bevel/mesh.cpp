#include "core/bevel/mesh.hpp"

#include "core/placement/placement.hpp"

namespace geargen::core::bevel::mesh {

Matrix3 pinion_placement() noexcept { return placement::pinion_placement(); }

Matrix3 gear_placement(double shaft_angle_rad, double clocking_rad) noexcept
{
    return placement::matmul(placement::rot_y(shaft_angle_rad),
                             placement::rot_z(clocking_rad));
}

double gear_clocking(int teeth) noexcept { return placement::gear_clocking(teeth); }

} // namespace geargen::core::bevel::mesh
