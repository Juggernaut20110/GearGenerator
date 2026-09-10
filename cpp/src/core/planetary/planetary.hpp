#pragma once

#include "core/common/parameters.hpp"

namespace geargen::core::planetary {

struct SetGeometry {
    PlanetaryParameters parameters;
    double centre_distance_mm{};
    double transverse_module_mm{};
    double transverse_pressure_angle_rad{};
    double circular_pitch_mm{};
    double axial_pitch_mm{};
    double whole_depth_mm{};
};

[[nodiscard]] PlanetaryParameters default_parameters(double module_mm,
                                                      int z_sun,
                                                      int z_planet);
[[nodiscard]] SetGeometry derive(const PlanetaryParameters& parameters);

} // namespace geargen::core::planetary
