#pragma once

#include "core/common/numerics.hpp"

#include <optional>
#include <string>

namespace geargen::core {

enum class Hand { Right, Left };
enum class SpurArrangement { External, Internal };

[[nodiscard]] const char* hand_name(Hand hand) noexcept;
[[nodiscard]] const char* arrangement_name(SpurArrangement arrangement) noexcept;

// These option records model Python's ``with_defaults(..., **overrides)``.
// An omitted optional means "apply the same default as Python"; a supplied
// value wins after the size-dependent defaults have been calculated.
struct SpurDefaultOverrides {
    std::optional<double> face_width;
    std::optional<double> bore;
    std::optional<double> hub_thickness;
    std::optional<double> pressure_angle;
    std::optional<double> helix_angle;
    std::optional<Hand> hand;
    std::optional<bool> internal;
    std::optional<double> fillet_factor;
    std::optional<double> backlash;
    std::optional<double> rim_thickness;
    std::optional<double> profile_shift_1;
    std::optional<double> profile_shift_2;
    std::optional<double> basic_rack_addendum_factor;
    std::optional<double> basic_rack_clearance_factor;
    std::optional<double> basic_rack_root_radius_factor;
    std::optional<double> working_centre_distance;
    std::optional<std::string> tip_alteration_mode;
    std::optional<double> tip_alteration_coefficient;
    std::optional<std::string> root_geometry;
    std::optional<std::string> backlash_mode;
    std::optional<double> backlash_allocation;
};

struct BevelDefaultOverrides {
    std::optional<double> face_width;
    std::optional<double> bore;
    std::optional<double> hub_thickness;
    std::optional<double> min_root_thickness;
    std::optional<double> pressure_angle;
    std::optional<double> shaft_angle;
    std::optional<double> spiral_angle;
    std::optional<Hand> hand;
    std::optional<double> cutter_radius;
    std::optional<double> fillet_factor;
    std::optional<double> backlash;
};

struct HypoidDefaultOverrides {
    std::optional<double> face_width;
    std::optional<double> bore;
    std::optional<double> hub_thickness;
    std::optional<double> pressure_angle;
    std::optional<double> shaft_angle;
    std::optional<double> offset;
    std::optional<double> spiral_angle;
    std::optional<Hand> hand;
    std::optional<double> cutter_radius;
    std::optional<double> backlash;
    std::optional<double> min_root_thickness;
    std::optional<double> gear_mean_addendum_factor;
    std::optional<double> depth_factor;
    std::optional<double> clearance_factor;
    std::optional<double> thickness_factor;
    std::optional<double> gear_addendum_angle;
    std::optional<double> gear_dedendum_angle;
    std::optional<double> root_fillet_radius;
};

struct PlanetaryDefaultOverrides {
    std::optional<int> n_planets;
    std::optional<double> face_width;
    std::optional<double> bore;
    std::optional<double> hub_thickness;
    std::optional<double> rim_thickness;
    std::optional<double> pressure_angle;
    std::optional<double> helix_angle;
    std::optional<Hand> hand;
    std::optional<double> fillet_factor;
    std::optional<double> backlash;
};

struct SpurSetParams {
    // Public/input boundary: lengths in mm, pressure/helix angles in degrees.
    // Geometry accessors below are the explicit degree->radian boundary.
    double module{};
    int z1{};
    int z2{};
    double face_width{};
    double bore{};
    double hub_thickness{};
    double pressure_angle{20.0};
    double helix_angle{};
    Hand hand{Hand::Right};
    bool internal{false};
    double fillet_factor{0.2};
    double backlash{};
    double rim_thickness{5.0};
    double profile_shift_1{};
    double profile_shift_2{};
    double basic_rack_addendum_factor{1.0};
    double basic_rack_clearance_factor{0.25};
    double basic_rack_root_radius_factor{0.38};
    std::optional<double> working_centre_distance;
    std::string tip_alteration_mode{"iso_clearance"};
    std::optional<double> tip_alteration_coefficient;
    std::string root_geometry{"legacy"};
    std::string backlash_mode{"legacy_reference"};
    double backlash_allocation{0.5};

    [[nodiscard]] static SpurSetParams with_defaults(
        double module, int z1, int z2,
        const SpurDefaultOverrides& overrides = {});
    [[nodiscard]] double alpha_n() const noexcept;
    [[nodiscard]] double beta() const noexcept;
    [[nodiscard]] double transverse_module() const noexcept;
    [[nodiscard]] double alpha_t() const noexcept;
    [[nodiscard]] double ratio() const noexcept;
    [[nodiscard]] double reference_centre_distance() const noexcept;
    [[nodiscard]] double profile_shift_combination() const noexcept;
    [[nodiscard]] double basic_rack_dedendum_factor() const noexcept;
};

struct BevelSetParams {
    // Outer transverse module/diameters use mm; all input angles are degrees.
    double module{};
    int z1{};
    int z2{};
    double face_width{};
    double bore{};
    double hub_thickness{};
    double min_root_thickness{0.5};
    double pressure_angle{20.0};
    double shaft_angle{90.0};
    double spiral_angle{};
    Hand hand{Hand::Right};
    std::optional<double> cutter_radius;
    double fillet_factor{0.2};
    double backlash{};

    [[nodiscard]] static BevelSetParams with_defaults(
        double module, int z1, int z2, bool zerol = false,
        const BevelDefaultOverrides& overrides = {});
    [[nodiscard]] double alpha() const noexcept;
    [[nodiscard]] double sigma() const noexcept;
    [[nodiscard]] double psi_m() const noexcept;
    [[nodiscard]] double trace_sign() const noexcept;
    [[nodiscard]] bool is_curved() const noexcept;
    [[nodiscard]] const char* trace_kind() const noexcept;
    [[nodiscard]] double ratio() const noexcept;
};

struct HypoidSetParams {
    // Public hypoid convention: module is wheel outer transverse module in mm;
    // offset is a signed mm axis displacement; spiral is a non-negative degree
    // magnitude whose sign comes from hand.
    double module{};
    int z1{};
    int z2{};
    double face_width{};
    double bore{};
    double hub_thickness{};
    double pressure_angle{20.0};
    double shaft_angle{90.0};
    double offset{};
    double spiral_angle{35.0};
    Hand hand{Hand::Right};
    std::optional<double> cutter_radius;
    double backlash{};
    double min_root_thickness{0.5};
    double gear_mean_addendum_factor{0.35};
    double depth_factor{2.0};
    double clearance_factor{0.125};
    double thickness_factor{0.10};
    double gear_addendum_angle{1.0};
    double gear_dedendum_angle{4.0};
    std::optional<double> root_fillet_radius;

    [[nodiscard]] static HypoidSetParams with_defaults(
        double module, int z1, int z2,
        const HypoidDefaultOverrides& overrides = {});
    [[nodiscard]] double alpha() const noexcept;
    [[nodiscard]] double sigma() const noexcept;
    [[nodiscard]] double spiral_sign() const noexcept;
    [[nodiscard]] double psi1() const noexcept;
    [[nodiscard]] double beta() const noexcept;
    [[nodiscard]] double ratio() const noexcept;
    [[nodiscard]] double wheel_outer_diameter() const noexcept;
    [[nodiscard]] double wheel_outer_radius() const noexcept;
    [[nodiscard]] double effective_root_fillet_radius() const noexcept;
};

struct PlanetarySetParams {
    // Public/input boundary: module and lengths in mm, angles in degrees.
    double module{};
    int z_sun{};
    int z_planet{};
    int n_planets{3};
    double face_width{20.0};
    double bore{10.0};
    double hub_thickness{5.0};
    double rim_thickness{5.0};
    double pressure_angle{20.0};
    double helix_angle{};
    Hand hand{Hand::Right};
    double fillet_factor{0.2};
    double backlash{};

    [[nodiscard]] static PlanetarySetParams with_defaults(
        double module, int z_sun, int z_planet,
        const PlanetaryDefaultOverrides& overrides = {});
    [[nodiscard]] int z_ring() const noexcept;
    [[nodiscard]] double alpha_n() const noexcept;
    [[nodiscard]] double beta() const noexcept;
    [[nodiscard]] double transverse_module() const noexcept;
    [[nodiscard]] double alpha_t() const noexcept;
    [[nodiscard]] double centre_distance() const noexcept;
    [[nodiscard]] int assembly_remainder() const noexcept;
    [[nodiscard]] double ratio_carrier_to_sun() const noexcept;
    [[nodiscard]] double ratio_ring_to_sun() const noexcept;
};

// Names retained for the first skeleton API. New code should use the explicit
// SetParams names, which match the Python dataclasses and avoid confusing input
// records with calculated geometry.
using SpurParameters = SpurSetParams;
using BevelParameters = BevelSetParams;
using HypoidParameters = HypoidSetParams;
using PlanetaryParameters = PlanetarySetParams;

} // namespace geargen::core
