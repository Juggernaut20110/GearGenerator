#pragma once

#include "core/common/geometry_types.hpp"

namespace geargen::core::involute {

struct ProfilePoint {
    Point2 point{};
    double roll_parameter{};
};

// Shared by external and internal gear profiles. The full port will add the
// rack-generated root, internal flank, fillet, and named tooth-space segments
// around this scalar primitive.
[[nodiscard]] double involute_function(double pressure_angle_rad) noexcept;

[[nodiscard]] ProfilePoint external_point(double base_radius_mm,
                                          double roll_parameter,
                                          double base_space_angle_rad);

} // namespace geargen::core::involute
