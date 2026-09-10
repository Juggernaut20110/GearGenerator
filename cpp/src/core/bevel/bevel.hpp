#pragma once

#include "core/common/parameters.hpp"

namespace geargen::core::bevel {

struct MemberGeometry {
    int teeth{};
    double pitch_angle_rad{};
    double outer_cone_distance_mm{};
    double inner_cone_distance_mm{};
    double virtual_teeth{};
};

struct SetGeometry {
    BevelParameters parameters;
    double outer_cone_dist_mm{};
    double mean_cone_dist_mm{};
    double inner_cone_dist_mm{};
    double section_scale{};
    double working_depth_mm{};
    double whole_depth_mm{};
    double clearance_mm{};
    double circular_pitch_mm{};
    MemberGeometry pinion;
    MemberGeometry gear;
};

[[nodiscard]] BevelParameters default_parameters(double module_mm, int z1, int z2,
                                                  bool zerol = false);
[[nodiscard]] SetGeometry derive(const BevelParameters& parameters);

} // namespace geargen::core::bevel
