#pragma once

#include <optional>
#include <string>

namespace geargen::core {

enum class Hand { Right, Left };
enum class SpurArrangement { External, Internal };

struct BevelParameters {
    double module_mm{};
    int z1{};
    int z2{};
    double face_width_mm{};
    double bore_mm{};
    double hub_thickness_mm{};
    double min_root_thickness_mm{0.5};
    double pressure_angle_deg{20.0};
    double shaft_angle_deg{90.0};
    double spiral_angle_deg{};
    Hand hand{Hand::Right};
    std::optional<double> cutter_radius_mm;
    double fillet_factor{0.2};
    double backlash_mm{};
};

struct SpurParameters {
    double module_mm{};
    int z1{};
    int z2{};
    double face_width_mm{};
    double bore_mm{};
    double hub_thickness_mm{};
    double pressure_angle_deg{20.0};
    double helix_angle_deg{};
    Hand hand{Hand::Right};
    SpurArrangement arrangement{SpurArrangement::External};
    double fillet_factor{0.2};
    double backlash_mm{};
    double rim_thickness_mm{5.0};
    double profile_shift_1{};
    double profile_shift_2{};
    std::optional<double> working_centre_distance_mm;
    std::string backlash_mode{"legacy_reference"};
    double backlash_allocation{0.5};
};

struct HypoidParameters {
    double module_mm{};
    int z1{};
    int z2{};
    double face_width_mm{};
    double bore_mm{};
    double hub_thickness_mm{};
    double pressure_angle_deg{20.0};
    double shaft_angle_deg{90.0};
    double offset_mm{};
    double spiral_angle_deg{35.0};
    Hand hand{Hand::Right};
    std::optional<double> cutter_radius_mm;
    double backlash_mm{};
    double min_root_thickness_mm{0.5};
    std::optional<double> root_fillet_radius_mm;
};

struct PlanetaryParameters {
    double module_mm{};
    int z_sun{};
    int z_planet{};
    int n_planets{3};
    double face_width_mm{20.0};
    double bore_mm{10.0};
    double hub_thickness_mm{5.0};
    double rim_thickness_mm{5.0};
    double pressure_angle_deg{20.0};
    double helix_angle_deg{};
    Hand hand{Hand::Right};
    double fillet_factor{0.2};
    double backlash_mm{};

    [[nodiscard]] int z_ring() const noexcept
    {
        return z_sun + 2 * z_planet;
    }
};

} // namespace geargen::core
