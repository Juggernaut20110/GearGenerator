#pragma once

#include "core/common/geometry_types.hpp"

#include <array>
#include <utility>

namespace geargen::core::placement {

// All placement angles are radians.  Placement consumes already-converted core
// geometry values; degrees are accepted only by the parameter records.
[[nodiscard]] double gear_clocking(int teeth) noexcept;
[[nodiscard]] double angular_velocity_ratio(int z1, int z2,
                                             bool internal = false) noexcept;
[[nodiscard]] std::pair<double, double> gear_mate_ratio(int z1,
                                                         int z2) noexcept;
[[nodiscard]] Matrix3 pinion_placement() noexcept;
[[nodiscard]] Matrix3 rot_y(double angle_rad) noexcept;
[[nodiscard]] Matrix3 rot_z(double angle_rad) noexcept;
[[nodiscard]] Matrix3 matmul(const Matrix3& lhs, const Matrix3& rhs) noexcept;
[[nodiscard]] Point3 apply(const Matrix3& matrix, Point3 vector) noexcept;
[[nodiscard]] std::array<double, 16> to_solidworks_array(
    const Matrix3& matrix, Point3 translation_m = {}, double scale = 1.0) noexcept;
[[nodiscard]] Point3 axis_of(const Matrix3& matrix) noexcept;
[[nodiscard]] double angle_between(Point3 lhs, Point3 rhs) noexcept;

} // namespace geargen::core::placement
