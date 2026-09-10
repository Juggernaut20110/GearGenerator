#include "core/planetary/planetary.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

geargen::core::SpurSetParams spur_parameters(
    const geargen::core::PlanetarySetParams& p, int z1, int z2,
    bool internal, geargen::core::Hand hand)
{
    auto result = geargen::core::SpurSetParams::with_defaults(p.module, z1, z2);
    result.face_width = p.face_width;
    result.bore = p.bore;
    result.hub_thickness = p.hub_thickness;
    result.pressure_angle = p.pressure_angle;
    result.helix_angle = p.helix_angle;
    result.hand = hand;
    result.internal = internal;
    result.fillet_factor = p.fillet_factor;
    result.backlash = p.backlash;
    result.rim_thickness = p.rim_thickness;
    return result;
}

} // namespace

namespace geargen::core::planetary {

PlanetaryParameters default_parameters(double module_mm, int z_sun,
                                        int z_planet)
{
    return PlanetaryParameters::with_defaults(module_mm, z_sun, z_planet);
}

SetGeometry compute_set(const PlanetarySetParams& p)
{
    const auto sun_planet_params =
        spur_parameters(p, p.z_sun, p.z_planet, false, p.hand);
    const auto opposite_hand = p.hand == Hand::Right ? Hand::Left : Hand::Right;
    const auto planet_ring_params = spur_parameters(
        p, p.z_planet, p.z_ring(), true, opposite_hand);
    const auto sun_planet = spur::compute_set(sun_planet_params);
    const auto planet_ring = spur::compute_set(planet_ring_params);
    return SetGeometry{
        p,
        sun_planet.working_centre_distance_mm,
        sun_planet.transverse_module_mm,
        sun_planet.reference_pressure_angle_rad,
        sun_planet.circular_pitch_mm,
        sun_planet.axial_pitch_mm,
        sun_planet.whole_depth_mm,
        sun_planet.pinion,
        sun_planet.gear,
        planet_ring.gear,
        sun_planet,
        planet_ring,
    };
}

SetGeometry derive(const PlanetarySetParams& parameters)
{
    return compute_set(parameters);
}

double centre_distance_from_ring(const PlanetarySetParams& p) noexcept
{
    return p.transverse_module() * (p.z_ring() - p.z_planet) / 2.0;
}

double SetGeometry::ring_rim_radius_mm() const
{
    return spur::rim_radius(planet_ring, "gear");
}

const spur::MemberGeometry& SetGeometry::member(const std::string& which) const
{
    if (which == kSun) {
        return sun;
    }
    if (which == kPlanet) {
        return planet;
    }
    if (which == kRing) {
        return ring;
    }
    throw std::invalid_argument("member must be sun, planet, or ring");
}

std::vector<double> SetGeometry::planet_angles_rad() const
{
    std::vector<double> result;
    result.reserve(static_cast<std::size_t>(parameters.n_planets));
    for (int index = 0; index < parameters.n_planets; ++index) {
        result.push_back(kTau * index / parameters.n_planets);
    }
    return result;
}

double SetGeometry::neighbour_spacing_mm() const noexcept
{
    if (parameters.n_planets < 2) {
        return std::numeric_limits<double>::infinity();
    }
    return 2.0 * centre_distance_mm *
           std::sin(kPi / parameters.n_planets);
}

const spur::SetGeometry& SetGeometry::mesh_for(const std::string& which) const
{
    if (which == kRing) {
        return planet_ring;
    }
    if (which == kSun || which == kPlanet) {
        return sun_planet;
    }
    throw std::invalid_argument("member must be sun, planet, or ring");
}

const char* SetGeometry::role_of(const std::string& which) const
{
    if (which == kSun) {
        return "pinion";
    }
    if (which == kPlanet || which == kRing) {
        return "gear";
    }
    throw std::invalid_argument("member must be sun, planet, or ring");
}

} // namespace geargen::core::planetary
