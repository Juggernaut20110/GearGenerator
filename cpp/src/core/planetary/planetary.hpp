#pragma once

#include "core/common/parameters.hpp"

namespace geargen::core::planetary {

struct SetGeometry {
    PlanetaryParameters parameters;
    double centre_distance_mm{};
};

[[nodiscard]] PlanetaryParameters default_parameters(double module_mm,
                                                      int z_sun,
                                                      int z_planet);

} // namespace geargen::core::planetary
