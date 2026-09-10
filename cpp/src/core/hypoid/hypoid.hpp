#pragma once

#include "core/common/parameters.hpp"

namespace geargen::core::hypoid {

struct MemberGeometry {
    int teeth{};
    double pitch_angle_rad{};
    double outer_tip_z_mm{};
};

struct SetGeometry {
    HypoidParameters parameters;
    MemberGeometry pinion;
    MemberGeometry gear;
    double pitch_plane_offset_mm{};
    double outer_cone_distance_mm{};
    double mean_cone_distance_mm{};
    double inner_cone_distance_mm{};
    double mean_normal_module_mm{};
};

[[nodiscard]] HypoidParameters default_parameters(double module_mm, int z1, int z2);
[[nodiscard]] SetGeometry derive(const HypoidParameters& parameters);

} // namespace geargen::core::hypoid
