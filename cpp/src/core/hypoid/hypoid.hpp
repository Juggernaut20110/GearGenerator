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
};

[[nodiscard]] HypoidParameters default_parameters(double module_mm, int z1, int z2);

} // namespace geargen::core::hypoid
