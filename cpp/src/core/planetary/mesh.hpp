#pragma once

#include "core/common/geometry_types.hpp"
#include "core/planetary/planetary.hpp"

#include <utility>

namespace geargen::core::planetary::mesh {

[[nodiscard]] double carrier_angle(const SetGeometry& geometry,
                                   int index) noexcept;
[[nodiscard]] double sun_clocking() noexcept;
[[nodiscard]] double planet_clocking(const SetGeometry& geometry,
                                     int index) noexcept;
[[nodiscard]] double ring_clocking(const SetGeometry& geometry,
                                   int index = 0) noexcept;
[[nodiscard]] Point3 planet_translation(const SetGeometry& geometry,
                                        int index) noexcept;
[[nodiscard]] Matrix3 member_placement(double clocking_rad) noexcept;
[[nodiscard]] std::pair<double, double> sun_planet_ratio(
    const SetGeometry& geometry) noexcept;
[[nodiscard]] std::pair<double, double> planet_ring_ratio(
    const SetGeometry& geometry) noexcept;

} // namespace geargen::core::planetary::mesh
