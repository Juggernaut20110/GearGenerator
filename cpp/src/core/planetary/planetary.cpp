#include "core/planetary/planetary.hpp"

#include <cmath>
#include <limits>

namespace geargen::core::planetary {

PlanetaryParameters default_parameters(double module_mm, int z_sun, int z_planet)
{
    return PlanetaryParameters::with_defaults(module_mm, z_sun, z_planet);
}

SetGeometry derive(const PlanetaryParameters& p)
{
    return {p,
            p.centre_distance(),
            p.transverse_module(),
            p.alpha_t(),
            kPi * p.transverse_module(),
            std::abs(std::sin(p.beta())) > 1e-12
                ? kPi * p.module / std::abs(std::sin(p.beta()))
                : std::numeric_limits<double>::infinity(),
            2.25 * p.module};
}

} // namespace geargen::core::planetary
