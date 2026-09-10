#include "core/validation/validation.hpp"

#include "core/common/numerics.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace geargen::core {

void ValidationResult::error(std::string field, std::string message)
{
    errors.push_back({std::move(field), std::move(message)});
}

void ValidationResult::warning(std::string field, std::string message)
{
    warnings.push_back({std::move(field), std::move(message)});
}

void ValidationResult::advisory(std::string field, std::string message)
{
    advisories.push_back({std::move(field), std::move(message)});
}

void validate_pair_basics(double module_mm, int z1, int z2,
                          double pressure_angle_deg,
                          ValidationResult& result)
{
    if (!std::isfinite(module_mm) || module_mm <= 0.0) {
        result.error("module", "must be greater than zero");
    }
    if (z1 < 6) {
        result.error("z1", "must be at least 6 teeth");
    }
    if (z2 < 6) {
        result.error("z2", "must be at least 6 teeth");
    }
    if (!std::isfinite(pressure_angle_deg) || pressure_angle_deg < 14.5 ||
        pressure_angle_deg > 25.0) {
        result.error("pressure_angle", "must be between 14.5 and 25.0 degrees");
    }
}

namespace {

constexpr int kMinTeeth = 6;
constexpr double kMinPressureAngle = 14.5;
constexpr double kMaxPressureAngle = 25.0;
constexpr double kMaxBacklashFraction = 0.05;
constexpr double kMaxHelixAngle = 45.0;
constexpr double kMaxBevelSpiralAngle = 45.0;
constexpr double kMaxHypoidSpiralAngle = 60.0;

bool valid_hand(Hand hand) noexcept
{
    return hand == Hand::Right || hand == Hand::Left;
}

void check_common_pair(double module, int z1, int z2, double pressure_angle,
                       ValidationResult& result)
{
    if (!finite(module)) {
        result.error("module", "must be finite");
    } else if (module <= 0.0) {
        result.error("module", "must be greater than zero");
    }
    if (z1 < kMinTeeth) {
        result.error("z1", "pinion needs at least 6 teeth");
    }
    if (z2 < kMinTeeth) {
        result.error("z2", "gear needs at least 6 teeth");
    }
    if (!finite(pressure_angle)) {
        result.error("pressure_angle", "must be finite");
    } else if (pressure_angle < kMinPressureAngle ||
               pressure_angle > kMaxPressureAngle) {
        result.error("pressure_angle", "must be between 14.5 and 25.0 degrees");
    }
}

void check_nonnegative_finite(const char* field, double value,
                              ValidationResult& result)
{
    if (!finite(value)) {
        result.error(field, "must be finite");
    } else if (value < 0.0) {
        result.error(field, "cannot be negative");
    }
}

double involute(double angle)
{
    return std::tan(angle) - angle;
}

double inverse_involute(double target, double lower, double upper)
{
    for (int i = 0; i < 80; ++i) {
        const double middle = (lower + upper) / 2.0;
        if (involute(middle) < target) {
            lower = middle;
        } else {
            upper = middle;
        }
    }
    return (lower + upper) / 2.0;
}

double working_pressure_angle(const SpurSetParams& p)
{
    if (p.working_centre_distance.has_value()) {
        const double ratio = p.reference_centre_distance() /
                             *p.working_centre_distance;
        return std::acos(std::clamp(ratio * std::cos(p.alpha_t()),
                                    -1.0, 1.0));
    }
    const double x = p.profile_shift_combination();
    if (std::abs(x) <= 1e-14 && !p.working_centre_distance.has_value()) {
        return p.alpha_t();
    }
    const double count = p.internal ? p.z2 - p.z1 : p.z1 + p.z2;
    const double target = involute(p.alpha_t()) +
                          2.0 * x * std::tan(p.alpha_n()) / count;
    return inverse_involute(target, 1e-10, kPi / 2.0 - 1e-8);
}

double calculate_working_distance(const SpurSetParams& p, double alpha_working)
{
    if (p.working_centre_distance.has_value()) {
        return *p.working_centre_distance;
    }
    const double base_1 = p.transverse_module() * p.z1 / 2.0 *
                          std::cos(p.alpha_t());
    const double base_2 = p.transverse_module() * p.z2 / 2.0 *
                          std::cos(p.alpha_t());
    const double working_1 = base_1 / std::cos(alpha_working);
    const double working_2 = base_2 / std::cos(alpha_working);
    return p.internal ? working_2 - working_1 : working_1 + working_2;
}

double tip_alteration(const SpurSetParams& p, double working_distance)
{
    if (p.tip_alteration_mode == "legacy") {
        return 0.0;
    }
    if (p.tip_alteration_mode == "explicit") {
        return p.tip_alteration_coefficient.value_or(0.0);
    }
    const double x = p.profile_shift_combination();
    const double distance_delta = working_distance -
                                   p.reference_centre_distance();
    return p.internal ? -distance_delta / p.module + x
                      : distance_delta / p.module - x;
}

void check_spur_basics(const SpurSetParams& p, ValidationResult& r)
{
    check_common_pair(p.module, p.z1, p.z2, p.pressure_angle, r);
    if (!finite(p.helix_angle) ||
        (finite(p.helix_angle) &&
         !(p.helix_angle > -kMaxHelixAngle && p.helix_angle < kMaxHelixAngle))) {
        r.error("helix_angle", "must be between -45 and 45 degrees (exclusive)");
    }
    if (!valid_hand(p.hand)) {
        r.error("hand", "must be 'right' or 'left'");
    }
    check_nonnegative_finite("face_width", p.face_width, r);
    if (finite(p.face_width) && p.face_width <= 0.0) {
        r.error("face_width", "must be greater than zero");
    }
    check_nonnegative_finite("bore", p.bore, r);
    check_nonnegative_finite("hub_thickness", p.hub_thickness, r);
    check_nonnegative_finite("backlash", p.backlash, r);
    check_nonnegative_finite("fillet_factor", p.fillet_factor, r);
    check_nonnegative_finite("rim_thickness", p.rim_thickness, r);
    if (p.internal && p.z2 <= p.z1) {
        r.error("z2", "an internal ring must have more teeth than the pinion");
    }
    for (const auto& item : {
             std::pair{"profile_shift_1", p.profile_shift_1},
             std::pair{"profile_shift_2", p.profile_shift_2},
             std::pair{"basic_rack_addendum_factor",
                       p.basic_rack_addendum_factor},
             std::pair{"basic_rack_clearance_factor",
                       p.basic_rack_clearance_factor},
             std::pair{"basic_rack_root_radius_factor",
                       p.basic_rack_root_radius_factor},
         }) {
        if (!finite(item.second)) {
            r.error(item.first, "must be finite");
        }
    }
    if (p.basic_rack_addendum_factor <= 0.0) {
        r.error("basic_rack_addendum_factor", "must be greater than zero");
    }
    if (p.basic_rack_clearance_factor < 0.0) {
        r.error("basic_rack_clearance_factor", "cannot be negative");
    }
    if (p.basic_rack_root_radius_factor < 0.0) {
        r.error("basic_rack_root_radius_factor", "cannot be negative");
    }
    if (p.internal && p.rim_thickness < 0.0) {
        r.error("rim_thickness", "cannot be negative");
    }
    if (p.backlash_mode != "legacy_reference" &&
        p.backlash_mode != "working_circumferential" &&
        p.backlash_mode != "normal") {
        r.error("backlash_mode",
                "must be one of legacy_reference, working_circumferential, normal");
    }
    check_nonnegative_finite("backlash_allocation", p.backlash_allocation, r);
    if (finite(p.backlash_allocation) && p.backlash_allocation > 1.0) {
        r.error("backlash_allocation", "must be between 0 and 1");
    }
    if (p.tip_alteration_mode != "legacy" &&
        p.tip_alteration_mode != "iso_clearance" &&
        p.tip_alteration_mode != "explicit") {
        r.error("tip_alteration_mode", "must be one of legacy, iso_clearance, explicit");
    }
    if (p.tip_alteration_mode == "explicit" &&
        !p.tip_alteration_coefficient.has_value()) {
        r.error("tip_alteration_coefficient",
                "is required when tip_alteration_mode is explicit");
    }
    if (p.tip_alteration_mode != "explicit" &&
        p.tip_alteration_coefficient.has_value()) {
        r.error("tip_alteration_coefficient",
                "is only accepted when tip_alteration_mode is explicit");
    }
    if (p.working_centre_distance.has_value() &&
        (!finite(*p.working_centre_distance) ||
         *p.working_centre_distance <= 0.0)) {
        r.error("working_centre_distance", "must be greater than zero");
    }
}

} // namespace

ValidationResult validate(const SpurSetParams& p)
{
    ValidationResult result;
    check_spur_basics(p, result);
    if (!result.ok()) {
        return result;
    }

    const double alpha_working = working_pressure_angle(p);
    if (!finite(alpha_working) || !(alpha_working > 0.0 &&
                                    alpha_working < kPi / 2.0)) {
        result.error("working_pressure_angle",
                     "must be finite and strictly between 0 and 90 degrees");
        return result;
    }
    const double working_distance =
        calculate_working_distance(p, alpha_working);

    const double circular_pitch = kPi * p.transverse_module();
    if (p.backlash > kMaxBacklashFraction * circular_pitch) {
        result.warning("backlash", "backlash is more than 5% of the circular pitch");
    }
    if (p.internal) {
        const double k = tip_alteration(p, working_distance);
        const double ring_reference =
            p.transverse_module() * p.z2 / 2.0;
        const double ring_base = ring_reference * std::cos(p.alpha_t());
        const double ring_tip = ring_reference -
            p.module * (p.basic_rack_addendum_factor - p.profile_shift_2 + k);
        if (ring_tip > ring_base && p.transverse_module() > 0.0) {
            const double alpha_tip = std::acos(
                std::clamp(ring_base / ring_tip, -1.0, 1.0));
            const double margin =
                static_cast<double>(p.z1) / p.z2 -
                (1.0 - std::tan(alpha_tip) / std::tan(alpha_working));
            if (margin < -kClearanceWarningTolerance) {
                std::ostringstream message;
                message << "tip-to-dedendum interference: CA >= CT1 "
                        << "(dimensionless margin " << std::setprecision(6)
                        << margin << ")";
                result.error("internal_tip_to_dedendum", message.str());
            }
        }
        result.warning(
            "internal_tip_to_dedendum",
            "ISO d_Nf2 < d_Ff2 check is unavailable: this internal member has "
            "no independently verified root-form diameter; nominal d_f is not "
            "substituted for d_Ff");
    }
    if (std::abs(p.beta()) <= 1e-12 && !p.internal) {
        const double sine = std::sin(p.alpha_t());
        const double effective_depth =
            p.basic_rack_dedendum_factor() -
            p.basic_rack_root_radius_factor * (1.0 - sine);
        const double limit = 2.0 * effective_depth / (sine * sine);
        if (p.z1 < limit) {
            std::ostringstream message;
            message << "pinion has " << p.z1
                    << " teeth, below the undercut limit of " << std::fixed
                    << std::setprecision(1) << limit << " for a "
                    << std::setprecision(1) << radians_to_degrees(p.alpha_t())
                    << " degree transverse pressure angle at x="
                    << std::defaultfloat << std::setprecision(3)
                    << p.profile_shift_1
                    << "; a real cutter would undercut the flank near the root, and "
                    << (p.root_geometry == "rack_generated"
                            ? "the selected rack-generated root shows that form limit"
                            : "the selected root approximation does not show the generated undercut");
            result.warning("z1", message.str());
        }
    }
    return result;
}

ValidationResult validate(const BevelSetParams& p)
{
    ValidationResult result;
    check_common_pair(p.module, p.z1, p.z2, p.pressure_angle, result);
    if (!finite(p.shaft_angle) || !(p.shaft_angle > 0.0 &&
                                    p.shaft_angle < 180.0)) {
        result.error("shaft_angle", "must be between 0 and 180 degrees (exclusive)");
    }
    check_nonnegative_finite("face_width", p.face_width, result);
    if (finite(p.face_width) && p.face_width <= 0.0) {
        result.error("face_width", "must be greater than zero");
    }
    check_nonnegative_finite("bore", p.bore, result);
    check_nonnegative_finite("hub_thickness", p.hub_thickness, result);
    check_nonnegative_finite("min_root_thickness", p.min_root_thickness, result);
    check_nonnegative_finite("backlash", p.backlash, result);
    if (!finite(p.spiral_angle) || std::abs(p.spiral_angle) > kMaxBevelSpiralAngle) {
        result.error("spiral_angle", "must be between -45 and 45 degrees");
    }
    if (!valid_hand(p.hand)) {
        result.error("hand", "must be 'right' or 'left'");
    }
    if (p.cutter_radius.has_value() &&
        (!finite(*p.cutter_radius) || *p.cutter_radius <= 0.0)) {
        result.error("cutter_radius", "must be greater than zero");
    }
    if (!result.ok()) {
        return result;
    }
    const double sigma = p.sigma();
    const double delta1 = std::atan2(std::sin(sigma),
                                     p.ratio() + std::cos(sigma));
    const double outer_cone = p.module * p.z1 / (2.0 * std::sin(delta1));
    const double inner = outer_cone - p.face_width;
    if (p.face_width >= outer_cone) {
        std::ostringstream message;
        message << "must be less than the outer cone distance ("
                << std::fixed << std::setprecision(2) << outer_cone << " mm)";
        result.error("face_width", message.str());
        return result;
    }
    const double limit = std::min((p.is_curved() ? 0.30 : 1.0 / 3.0) * outer_cone,
                                  10.0 * p.module);
    if (p.face_width > limit) {
        result.warning("face_width", "exceeds the usual bevel face-width limit");
    }
    if (p.is_curved() && std::abs(p.spiral_angle) <= 1e-12 &&
        p.cutter_radius.has_value()) {
        const double mean = outer_cone - p.face_width / 2.0;
        const double cutter = *p.cutter_radius;
        const double rho = std::sqrt(
            std::max(0.0, mean * mean + cutter * cutter));
        const auto angle_at = [&](double cone_distance) {
            const double sine = (cone_distance * cone_distance +
                                 cutter * cutter - rho * rho) /
                                (2.0 * cone_distance * cutter);
            return radians_to_degrees(
                p.trace_sign() * std::asin(std::clamp(sine, -1.0, 1.0)));
        };
        const double toe = angle_at(inner);
        const double heel = angle_at(outer_cone);
        const double swing = std::abs(heel - toe);
        if (swing > 20.0) {
            std::ostringstream message;
            message << "the spiral angle runs " << std::fixed
                    << std::setprecision(1) << std::abs(toe)
                    << " deg at the toe to " << std::abs(heel)
                    << " at the heel, a " << swing
                    << " deg swing; a cutter larger than the "
                    << std::setprecision(2) << cutter
                    << " mm one would flatten it";
            result.warning("cutter_radius", message.str());
        }
    }
    if (std::abs(p.spiral_angle) > 40.0) {
        result.warning("spiral_angle", "high spiral angle increases axial thrust");
    }
    return result;
}

ValidationResult validate(const HypoidSetParams& p)
{
    ValidationResult result;
    check_common_pair(p.module, p.z1, p.z2, p.pressure_angle, result);
    if (!finite(p.shaft_angle) || !(p.shaft_angle > 0.0 &&
                                    p.shaft_angle < 180.0)) {
        result.error("shaft_angle", "must be between 0 and 180 degrees (exclusive)");
    }
    check_nonnegative_finite("face_width", p.face_width, result);
    if (finite(p.face_width) && p.face_width <= 0.0) {
        result.error("face_width", "must be greater than zero");
    }
    check_nonnegative_finite("bore", p.bore, result);
    check_nonnegative_finite("hub_thickness", p.hub_thickness, result);
    check_nonnegative_finite("min_root_thickness", p.min_root_thickness, result);
    check_nonnegative_finite("backlash", p.backlash, result);
    if (!finite(p.spiral_angle) || p.spiral_angle < 0.0 ||
        p.spiral_angle > kMaxHypoidSpiralAngle) {
        result.error("spiral_angle", "must be a finite non-negative magnitude between 0 and 60 degrees");
    }
    if (!valid_hand(p.hand)) {
        result.error("hand", "must be 'right' or 'left'");
    }
    if (p.cutter_radius.has_value() &&
        (!finite(*p.cutter_radius) || *p.cutter_radius <= 0.0)) {
        result.error("cutter_radius", "must be greater than zero");
    }
    if (std::abs(p.offset) > p.wheel_outer_diameter() * 0.25) {
        result.error("offset", "must not exceed 25% of the wheel outer diameter");
    }
    if (!result.ok()) {
        return result;
    }
    if (std::abs(p.spiral_angle) > 40.0) {
        result.warning("spiral_angle", "high spiral angle increases axial thrust");
    }
    return result;
}

ValidationResult validate(const PlanetarySetParams& p)
{
    ValidationResult result;
    check_common_pair(p.module, p.z_sun, p.z_planet, p.pressure_angle, result);
    if (!finite(p.helix_angle) ||
        (finite(p.helix_angle) &&
         !(p.helix_angle > -kMaxHelixAngle && p.helix_angle < kMaxHelixAngle))) {
        result.error("helix_angle", "must be between -45 and 45 degrees (exclusive)");
    }
    if (!valid_hand(p.hand)) {
        result.error("hand", "must be 'right' or 'left'");
    }
    if (p.n_planets < 1 || p.n_planets > 12) {
        result.error("n_planets", "must be between 1 and 12");
    }
    check_nonnegative_finite("face_width", p.face_width, result);
    if (finite(p.face_width) && p.face_width <= 0.0) {
        result.error("face_width", "must be greater than zero");
    }
    check_nonnegative_finite("bore", p.bore, result);
    check_nonnegative_finite("hub_thickness", p.hub_thickness, result);
    check_nonnegative_finite("rim_thickness", p.rim_thickness, result);
    check_nonnegative_finite("backlash", p.backlash, result);
    if (!result.ok()) {
        return result;
    }
    if (p.assembly_remainder() != 0) {
        result.error("n_planets", "planet phases do not satisfy the assembly condition");
    }

    // Both planetary meshes use the same spur model. This first foundation
    // checks the internal ISO interference relationship without pretending to
    // own the full three-member tooth/loft geometry yet.
    SpurSetParams ring_mesh = SpurSetParams::with_defaults(p.module, p.z_planet,
                                                            p.z_ring());
    ring_mesh.face_width = p.face_width;
    ring_mesh.bore = p.bore;
    ring_mesh.hub_thickness = p.hub_thickness;
    ring_mesh.pressure_angle = p.pressure_angle;
    ring_mesh.helix_angle = p.helix_angle;
    ring_mesh.hand = p.hand == Hand::Right ? Hand::Left : Hand::Right;
    ring_mesh.internal = true;
    ring_mesh.fillet_factor = p.fillet_factor;
    ring_mesh.backlash = p.backlash;
    const auto internal_result = validate(ring_mesh);
    result.errors.insert(result.errors.end(), internal_result.errors.begin(),
                         internal_result.errors.end());
    result.warnings.insert(result.warnings.end(), internal_result.warnings.begin(),
                           internal_result.warnings.end());
    return result;
}

} // namespace geargen::core
