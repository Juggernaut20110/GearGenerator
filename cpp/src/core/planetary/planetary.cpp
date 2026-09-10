#include "core/planetary/planetary.hpp"

#include <algorithm>
#include <cmath>

namespace geargen::core::planetary {

PlanetaryParameters default_parameters(double module_mm, int z_sun, int z_planet)
{
    PlanetaryParameters result;
    result.module_mm = module_mm;
    result.z_sun = z_sun;
    result.z_planet = z_planet;
    result.face_width_mm = 10.0 * module_mm;
    result.bore_mm = std::max(0.25 * module_mm * static_cast<double>(z_sun), 4.0);
    result.hub_thickness_mm = 2.5 * module_mm;
    result.rim_thickness_mm = 2.5 * module_mm;
    return result;
}

} // namespace geargen::core::planetary
