#pragma once

#include "core/common/geometry_types.hpp"
#include "core/spur/spur.hpp"

#include <utility>

namespace geargen::core::spur::mesh {

[[nodiscard]] Matrix3 gear_placement(double clocking_rad) noexcept;
[[nodiscard]] Point3 gear_translation(const SetGeometry& geometry) noexcept;
[[nodiscard]] double internal_gear_clocking(int teeth) noexcept;
[[nodiscard]] double clocking_for(const SetGeometry& geometry) noexcept;
[[nodiscard]] std::pair<double, double> mate_ratio(const SetGeometry& geometry) noexcept;

} // namespace geargen::core::spur::mesh
