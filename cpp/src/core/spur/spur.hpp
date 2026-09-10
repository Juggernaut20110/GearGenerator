#pragma once

#include "core/common/parameters.hpp"

namespace geargen::core::spur {

struct MemberGeometry {
    int teeth{};
    double beta_rad{};
    double reference_radius_mm{};
    double base_radius_mm{};
    double working_radius_mm{};
    double tip_radius_mm{};
    double root_radius_mm{};
    double twist_rad{};
    bool internal{};
};

struct SetGeometry {
    SpurParameters parameters;
    double transverse_module_mm{};
    double reference_pressure_angle_rad{};
    double working_pressure_angle_rad{};
    double working_centre_distance_mm{};
    double circular_pitch_mm{};
    double whole_depth_mm{};
    MemberGeometry pinion;
    MemberGeometry gear;
};

[[nodiscard]] SpurParameters default_parameters(double module_mm, int z1, int z2);
[[nodiscard]] SetGeometry derive(const SpurParameters& parameters);

} // namespace geargen::core::spur
