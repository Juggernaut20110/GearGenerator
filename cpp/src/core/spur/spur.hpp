#pragma once

#include "core/common/parameters.hpp"

namespace geargen::core::spur {

struct MemberGeometry {
    int teeth{};
    double reference_radius_mm{};
    double base_radius_mm{};
    double tip_radius_mm{};
    double root_radius_mm{};
    double twist_rad{};
};

struct SetGeometry {
    SpurParameters parameters;
    double transverse_module_mm{};
    double working_centre_distance_mm{};
    MemberGeometry pinion;
    MemberGeometry gear;
};

[[nodiscard]] SpurParameters default_parameters(double module_mm, int z1, int z2);

} // namespace geargen::core::spur
