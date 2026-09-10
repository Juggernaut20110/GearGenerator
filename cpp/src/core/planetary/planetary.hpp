#pragma once

#include "core/common/parameters.hpp"
#include "core/spur/spur.hpp"

#include <limits>
#include <string>
#include <vector>

namespace geargen::core::planetary {

inline constexpr const char* kSun = "sun";
inline constexpr const char* kPlanet = "planet";
inline constexpr const char* kRing = "ring";

struct SetGeometry {
    PlanetaryParameters parameters;
    double centre_distance_mm{};
    double transverse_module_mm{};
    double transverse_pressure_angle_rad{};
    double circular_pitch_mm{};
    double axial_pitch_mm{};
    double whole_depth_mm{};
    spur::MemberGeometry sun;
    spur::MemberGeometry planet;
    spur::MemberGeometry ring;
    spur::SetGeometry sun_planet;
    spur::SetGeometry planet_ring;

    [[nodiscard]] double ring_rim_radius_mm() const;
    [[nodiscard]] std::vector<double> planet_angles_rad() const;
    [[nodiscard]] double neighbour_spacing_mm() const noexcept;
    [[nodiscard]] double sun_planet_contact_ratio() const noexcept
    {
        return sun_planet.total_contact_ratio();
    }
    [[nodiscard]] double planet_ring_contact_ratio() const noexcept
    {
        return planet_ring.total_contact_ratio();
    }
    [[nodiscard]] const spur::MemberGeometry& member(
        const std::string& which) const;
    [[nodiscard]] const spur::SetGeometry& mesh_for(
        const std::string& which) const;
    [[nodiscard]] const char* role_of(const std::string& which) const;
};

[[nodiscard]] PlanetaryParameters default_parameters(double module_mm,
                                                      int z_sun,
                                                      int z_planet);
[[nodiscard]] SetGeometry compute_set(const PlanetarySetParams& parameters);
[[nodiscard]] SetGeometry derive(const PlanetarySetParams& parameters);
[[nodiscard]] double centre_distance_from_ring(
    const PlanetarySetParams& parameters) noexcept;

} // namespace geargen::core::planetary
