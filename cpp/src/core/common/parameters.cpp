#include "core/common/parameters.hpp"

#include <algorithm>
#include <cmath>

namespace geargen::core {

namespace {

template <typename T>
void apply_if(const std::optional<T>& value, T& target)
{
    if (value.has_value()) {
        target = *value;
    }
}

double rounded(double value, int digits)
{
    const double scale = std::pow(10.0, static_cast<double>(digits));
    return std::round(value * scale) / scale;
}

} // namespace

const char* hand_name(Hand hand) noexcept
{
    return hand == Hand::Right ? "right" : "left";
}

const char* arrangement_name(SpurArrangement arrangement) noexcept
{
    return arrangement == SpurArrangement::External ? "external" : "internal";
}

SpurSetParams SpurSetParams::with_defaults(double module_value, int z1_value,
                                           int z2_value,
                                           const SpurDefaultOverrides& o)
{
    SpurSetParams result;
    result.module = module_value;
    result.z1 = z1_value;
    result.z2 = z2_value;

    const double helix_deg = o.helix_angle.value_or(0.0);
    const double beta_for_sizing = degrees_to_radians(helix_deg);
    double face = 10.0 * module_value;
    if (std::abs(std::sin(beta_for_sizing)) > 1e-9) {
        face = std::max(face, kPi * module_value /
                                  std::abs(std::sin(beta_for_sizing)));
    }
    const double transverse_module_value =
        module_value / std::cos(beta_for_sizing);
    result.face_width = rounded(face, 2);
    result.bore = rounded(std::max(0.25 * transverse_module_value * z1_value,
                                   4.0), 1);
    result.hub_thickness = rounded(2.5 * module_value, 2);
    result.rim_thickness = rounded(2.5 * module_value, 2);

    apply_if(o.face_width, result.face_width);
    apply_if(o.bore, result.bore);
    apply_if(o.hub_thickness, result.hub_thickness);
    apply_if(o.pressure_angle, result.pressure_angle);
    apply_if(o.helix_angle, result.helix_angle);
    apply_if(o.hand, result.hand);
    if (o.internal.has_value()) {
        result.internal = *o.internal;
    }
    apply_if(o.fillet_factor, result.fillet_factor);
    apply_if(o.backlash, result.backlash);
    apply_if(o.rim_thickness, result.rim_thickness);
    apply_if(o.profile_shift_1, result.profile_shift_1);
    apply_if(o.profile_shift_2, result.profile_shift_2);
    apply_if(o.basic_rack_addendum_factor,
             result.basic_rack_addendum_factor);
    apply_if(o.basic_rack_clearance_factor,
             result.basic_rack_clearance_factor);
    apply_if(o.basic_rack_root_radius_factor,
             result.basic_rack_root_radius_factor);
    if (o.working_centre_distance.has_value()) {
        result.working_centre_distance = o.working_centre_distance;
    }
    if (o.tip_alteration_mode.has_value()) {
        result.tip_alteration_mode = *o.tip_alteration_mode;
    }
    if (o.tip_alteration_coefficient.has_value()) {
        result.tip_alteration_coefficient = o.tip_alteration_coefficient;
    }
    if (o.root_geometry.has_value()) {
        result.root_geometry = *o.root_geometry;
    }
    if (o.backlash_mode.has_value()) {
        result.backlash_mode = *o.backlash_mode;
    }
    apply_if(o.backlash_allocation, result.backlash_allocation);
    return result;
}

double SpurSetParams::alpha_n() const noexcept
{
    return degrees_to_radians(pressure_angle);
}

double SpurSetParams::beta() const noexcept
{
    return signed_angle(helix_angle, hand == Hand::Right);
}

double SpurSetParams::transverse_module() const noexcept
{
    return module / std::cos(beta());
}

double SpurSetParams::alpha_t() const noexcept
{
    return std::atan2(std::tan(alpha_n()), std::cos(beta()));
}

double SpurSetParams::ratio() const noexcept
{
    return static_cast<double>(z2) / static_cast<double>(z1);
}

double SpurSetParams::reference_centre_distance() const noexcept
{
    const int count_difference = internal ? z2 - z1 : z1 + z2;
    return transverse_module() * static_cast<double>(count_difference) / 2.0;
}

double SpurSetParams::profile_shift_combination() const noexcept
{
    return internal ? profile_shift_2 - profile_shift_1
                    : profile_shift_1 + profile_shift_2;
}

double SpurSetParams::basic_rack_dedendum_factor() const noexcept
{
    return basic_rack_addendum_factor + basic_rack_clearance_factor;
}

BevelSetParams BevelSetParams::with_defaults(double module_value, int z1_value,
                                             int z2_value, bool zerol,
                                             const BevelDefaultOverrides& o)
{
    BevelSetParams result;
    result.module = module_value;
    result.z1 = z1_value;
    result.z2 = z2_value;
    const double shaft_angle_deg = o.shaft_angle.value_or(90.0);
    const double sigma_value = degrees_to_radians(shaft_angle_deg);
    const double delta1 = std::atan2(
        std::sin(sigma_value), static_cast<double>(z2_value) / z1_value +
                                  std::cos(sigma_value));
    const double outer_cone_distance =
        module_value * static_cast<double>(z1_value) /
        (2.0 * std::sin(delta1));
    const bool curved =
        zerol || std::abs(o.spiral_angle.value_or(0.0)) > 1e-12 ||
        o.cutter_radius.has_value();
    result.face_width = rounded(
        std::min((curved ? 0.30 : 1.0 / 3.0) * outer_cone_distance,
                 10.0 * module_value),
        2);
    result.bore = rounded(std::max(0.25 * module_value * z1_value, 4.0), 1);
    result.hub_thickness = rounded(2.5 * module_value, 2);
    result.min_root_thickness = rounded(0.25 * module_value, 2);

    apply_if(o.face_width, result.face_width);
    apply_if(o.bore, result.bore);
    apply_if(o.hub_thickness, result.hub_thickness);
    apply_if(o.min_root_thickness, result.min_root_thickness);
    apply_if(o.pressure_angle, result.pressure_angle);
    apply_if(o.shaft_angle, result.shaft_angle);
    apply_if(o.spiral_angle, result.spiral_angle);
    apply_if(o.hand, result.hand);
    if (o.cutter_radius.has_value()) {
        result.cutter_radius = o.cutter_radius;
    } else if (curved) {
        result.cutter_radius = rounded(
            outer_cone_distance - result.face_width / 2.0, 4);
    }
    apply_if(o.fillet_factor, result.fillet_factor);
    apply_if(o.backlash, result.backlash);
    return result;
}

double BevelSetParams::alpha() const noexcept
{
    return degrees_to_radians(pressure_angle);
}

double BevelSetParams::sigma() const noexcept
{
    return degrees_to_radians(shaft_angle);
}

double BevelSetParams::psi_m() const noexcept
{
    return signed_angle(spiral_angle, hand == Hand::Right);
}

double BevelSetParams::trace_sign() const noexcept
{
    if (std::abs(spiral_angle) > 1e-12) {
        return psi_m() < 0.0 ? -1.0 : 1.0;
    }
    return hand == Hand::Right ? 1.0 : -1.0;
}

bool BevelSetParams::is_curved() const noexcept
{
    return std::abs(spiral_angle) > 1e-12 || cutter_radius.has_value();
}

const char* BevelSetParams::trace_kind() const noexcept
{
    if (!is_curved()) {
        return "straight";
    }
    return std::abs(spiral_angle) > 1e-12 ? "spiral" : "zerol";
}

double BevelSetParams::ratio() const noexcept
{
    return static_cast<double>(z2) / static_cast<double>(z1);
}

HypoidSetParams HypoidSetParams::with_defaults(double module_value,
                                               int z1_value, int z2_value,
                                               const HypoidDefaultOverrides& o)
{
    HypoidSetParams result;
    result.module = module_value;
    result.z1 = z1_value;
    result.z2 = z2_value;
    result.face_width = rounded(
        std::min(0.18 * module_value * z2_value, 10.0 * module_value), 2);
    result.bore = rounded(std::max(0.22 * module_value * z1_value, 4.0), 1);
    result.hub_thickness = rounded(2.5 * module_value, 2);
    result.cutter_radius = rounded(0.375 * module_value * z2_value, 4);
    result.min_root_thickness = rounded(0.25 * module_value, 2);

    apply_if(o.face_width, result.face_width);
    apply_if(o.bore, result.bore);
    apply_if(o.hub_thickness, result.hub_thickness);
    apply_if(o.pressure_angle, result.pressure_angle);
    apply_if(o.shaft_angle, result.shaft_angle);
    apply_if(o.offset, result.offset);
    apply_if(o.spiral_angle, result.spiral_angle);
    apply_if(o.hand, result.hand);
    if (o.cutter_radius.has_value()) {
        result.cutter_radius = o.cutter_radius;
    }
    apply_if(o.backlash, result.backlash);
    apply_if(o.min_root_thickness, result.min_root_thickness);
    apply_if(o.gear_mean_addendum_factor,
             result.gear_mean_addendum_factor);
    apply_if(o.depth_factor, result.depth_factor);
    apply_if(o.clearance_factor, result.clearance_factor);
    apply_if(o.thickness_factor, result.thickness_factor);
    apply_if(o.gear_addendum_angle, result.gear_addendum_angle);
    apply_if(o.gear_dedendum_angle, result.gear_dedendum_angle);
    if (o.root_fillet_radius.has_value()) {
        result.root_fillet_radius = o.root_fillet_radius;
    }
    return result;
}

double HypoidSetParams::alpha() const noexcept
{
    return degrees_to_radians(pressure_angle);
}

double HypoidSetParams::sigma() const noexcept
{
    return degrees_to_radians(shaft_angle);
}

double HypoidSetParams::spiral_sign() const noexcept
{
    return hand == Hand::Right ? 1.0 : -1.0;
}

double HypoidSetParams::psi1() const noexcept
{
    return spiral_sign() * degrees_to_radians(spiral_angle);
}

double HypoidSetParams::beta() const noexcept
{
    return psi1();
}

double HypoidSetParams::ratio() const noexcept
{
    return static_cast<double>(z2) / static_cast<double>(z1);
}

double HypoidSetParams::wheel_outer_diameter() const noexcept
{
    return module * static_cast<double>(z2);
}

double HypoidSetParams::wheel_outer_radius() const noexcept
{
    return wheel_outer_diameter() / 2.0;
}

double HypoidSetParams::effective_root_fillet_radius() const noexcept
{
    return root_fillet_radius.value_or(0.1 * module);
}

PlanetarySetParams PlanetarySetParams::with_defaults(
    double module_value, int z_sun_value, int z_planet_value,
    const PlanetaryDefaultOverrides& o)
{
    PlanetarySetParams result;
    result.module = module_value;
    result.z_sun = z_sun_value;
    result.z_planet = z_planet_value;
    result.n_planets = o.n_planets.value_or(3);
    const double beta_for_sizing =
        degrees_to_radians(o.helix_angle.value_or(0.0));
    double face = 10.0 * module_value;
    if (std::abs(std::sin(beta_for_sizing)) > 1e-9) {
        face = std::max(face, kPi * module_value /
                                  std::abs(std::sin(beta_for_sizing)));
    }
    const double transverse_module_value =
        module_value / std::cos(beta_for_sizing);
    result.face_width = rounded(face, 2);
    result.bore = rounded(
        std::max(0.25 * transverse_module_value * z_sun_value, 4.0), 1);
    result.hub_thickness = rounded(2.5 * module_value, 2);
    result.rim_thickness = rounded(2.5 * module_value, 2);

    apply_if(o.n_planets, result.n_planets);
    apply_if(o.face_width, result.face_width);
    apply_if(o.bore, result.bore);
    apply_if(o.hub_thickness, result.hub_thickness);
    apply_if(o.rim_thickness, result.rim_thickness);
    apply_if(o.pressure_angle, result.pressure_angle);
    apply_if(o.helix_angle, result.helix_angle);
    apply_if(o.hand, result.hand);
    apply_if(o.fillet_factor, result.fillet_factor);
    apply_if(o.backlash, result.backlash);
    return result;
}

int PlanetarySetParams::z_ring() const noexcept
{
    return z_sun + 2 * z_planet;
}

double PlanetarySetParams::alpha_n() const noexcept
{
    return degrees_to_radians(pressure_angle);
}

double PlanetarySetParams::beta() const noexcept
{
    return signed_angle(helix_angle, hand == Hand::Right);
}

double PlanetarySetParams::transverse_module() const noexcept
{
    return module / std::cos(beta());
}

double PlanetarySetParams::alpha_t() const noexcept
{
    return std::atan2(std::tan(alpha_n()), std::cos(beta()));
}

double PlanetarySetParams::centre_distance() const noexcept
{
    return transverse_module() * static_cast<double>(z_sun + z_planet) / 2.0;
}

int PlanetarySetParams::assembly_remainder() const noexcept
{
    return (z_sun + z_ring()) % n_planets;
}

double PlanetarySetParams::ratio_carrier_to_sun() const noexcept
{
    return 1.0 + static_cast<double>(z_ring()) / z_sun;
}

double PlanetarySetParams::ratio_ring_to_sun() const noexcept
{
    return -static_cast<double>(z_ring()) / z_sun;
}

} // namespace geargen::core
