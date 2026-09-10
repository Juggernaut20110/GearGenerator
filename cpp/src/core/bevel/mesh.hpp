#pragma once

#include "core/common/geometry_types.hpp"

namespace geargen::core::bevel::mesh {

[[nodiscard]] Matrix3 pinion_placement() noexcept;
[[nodiscard]] Matrix3 gear_placement(double shaft_angle_rad,
                                      double clocking_rad) noexcept;
[[nodiscard]] double gear_clocking(int teeth) noexcept;

} // namespace geargen::core::bevel::mesh
