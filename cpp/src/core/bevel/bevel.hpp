#pragma once

#include "core/common/parameters.hpp"

namespace geargen::core::bevel {

struct MemberGeometry {
    int teeth{};
    double pitch_angle_rad{};
    double outer_cone_distance_mm{};
    double inner_cone_distance_mm{};
};

struct SetGeometry {
    BevelParameters parameters;
    MemberGeometry pinion;
    MemberGeometry gear;
};

[[nodiscard]] BevelParameters default_parameters(double module_mm, int z1, int z2,
                                                  bool zerol = false);

} // namespace geargen::core::bevel
