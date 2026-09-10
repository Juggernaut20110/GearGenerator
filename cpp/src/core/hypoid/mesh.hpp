#pragma once

#include "core/common/geometry_types.hpp"
#include "core/hypoid/hypoid.hpp"

namespace geargen::core::hypoid::mesh {

[[nodiscard]] Matrix3 pinion_placement(const SetGeometry& geo) noexcept;
[[nodiscard]] double gear_clocking(const SetGeometry& geo) noexcept;
[[nodiscard]] Matrix3 gear_placement(double shaft_angle_rad,
                                      double clocking_rad) noexcept;
[[nodiscard]] Point3 gear_translation(const SetGeometry& geo);
[[nodiscard]] Point3 contact_point(const SetGeometry& geo);
[[nodiscard]] Point3 gear_contact_point(const SetGeometry& geo);
[[nodiscard]] double axis_offset(const SetGeometry& geo) noexcept;
[[nodiscard]] double skew_axis_distance(Point3 origin1, Point3 axis1,
                                        Point3 origin2, Point3 axis2) noexcept;

} // namespace geargen::core::hypoid::mesh
