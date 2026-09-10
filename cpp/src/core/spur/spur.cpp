#include "core/spur/spur.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

using geargen::core::Point2;
using geargen::core::Point3;
using geargen::core::SpurSetParams;
using geargen::core::involute::inv;

double inverse_involute(double value)
{
    if (value < -1e-14) {
        throw std::invalid_argument(
            "working involute is below the physical alpha = 0 domain");
    }
    if (std::abs(value) <= 1e-14) {
        return 0.0;
    }
    double lo = 0.0;
    double hi = std::nextafter(geargen::core::kPi / 2.0, 0.0);
    if (inv(hi) < value) {
        throw std::invalid_argument(
            "working involute is outside the physical alpha domain");
    }
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (inv(mid) < value) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

std::pair<double, double> working_geometry(const SpurSetParams& p,
                                           double reference_distance,
                                           double reference_alpha)
{
    if (p.working_centre_distance.has_value()) {
        const double working_distance = *p.working_centre_distance;
        if (working_distance <= 0.0) {
            throw std::invalid_argument(
                "working centre distance must be greater than zero");
        }
        double cosine = reference_distance / working_distance *
                        std::cos(reference_alpha);
        if (cosine <= 0.0 || cosine > 1.0 + 1e-12) {
            throw std::invalid_argument(
                "working centre distance gives a non-physical pressure angle");
        }
        cosine = std::min(1.0, cosine);
        return {std::acos(cosine), working_distance};
    }
    const double combination = p.profile_shift_combination();
    if (std::abs(combination) <= 1e-15) {
        return {reference_alpha, reference_distance};
    }
    const int count = p.internal ? p.z2 - p.z1 : p.z1 + p.z2;
    const double target = inv(reference_alpha) +
                          2.0 * combination * std::tan(p.alpha_n()) / count;
    const double working_alpha = inverse_involute(target);
    return {working_alpha,
            reference_distance * std::cos(reference_alpha) /
                std::cos(working_alpha)};
}

double working_helix_angle(double beta, double alpha) noexcept
{
    return std::atan(std::tan(std::abs(beta)) * std::cos(alpha));
}

std::optional<double> working_backlash_target(const SpurSetParams& p,
                                              double working_alpha)
{
    if (p.backlash_mode == "legacy_reference") {
        return std::nullopt;
    }
    if (p.backlash_mode == "working_circumferential") {
        return p.backlash;
    }
    if (p.backlash_mode == "normal") {
        const double beta_b = working_helix_angle(p.beta(), p.alpha_t());
        return p.backlash /
               (std::cos(working_alpha) * std::cos(beta_b));
    }
    throw std::invalid_argument(
        "backlash_mode must be legacy_reference, working_circumferential, or normal");
}

double tooth_width_at_radius(bool internal, double radius, double base_radius,
                             double psi0, double half_pitch)
{
    if (internal) {
        return geargen::core::involute::internal_tooth_width(
            radius, base_radius, psi0, half_pitch);
    }
    const double alpha = std::acos(std::min(1.0, base_radius / radius));
    return 2.0 * radius * (psi0 - inv(alpha));
}

double root_clearance_radius(const geargen::core::spur::MemberGeometry& member)
{
    return member.generated_root_radius_mm.value_or(member.root_radius_mm);
}

std::optional<double> involute_line_offset(double radius, double base_radius)
{
    if (radius < base_radius - 1e-10 * std::max(1.0, base_radius)) {
        return std::nullopt;
    }
    return std::sqrt(std::max(0.0, radius * radius - base_radius * base_radius));
}

double radius_from_offset(double base_radius, double offset) noexcept
{
    return std::sqrt(base_radius * base_radius +
                     std::max(0.0, offset) * std::max(0.0, offset));
}

void resolve_active_profile_geometry(
    geargen::core::spur::MemberGeometry& pinion,
    geargen::core::spur::MemberGeometry& gear)
{
    auto process = [](const geargen::core::spur::MemberGeometry& member) {
        const auto q_work =
            involute_line_offset(member.working_radius_mm, member.base_radius_mm);
        const double tip_form = member.tip_form_radius_mm;
        const auto q_tip = involute_line_offset(tip_form, member.base_radius_mm);
        return std::pair{q_work, q_tip};
    };
    const auto [q_work_1, q_tip_1] = process(pinion);
    const auto [q_work_2, q_tip_2] = process(gear);
    if (!q_work_1.has_value() || !q_tip_1.has_value() ||
        !q_work_2.has_value() || !q_tip_2.has_value()) {
        pinion.start_active_profile_radius_mm.reset();
        pinion.active_tip_radius_mm.reset();
        gear.start_active_profile_radius_mm.reset();
        gear.active_tip_radius_mm.reset();
        return;
    }

    const bool form_known = pinion.root_form_radius_mm.has_value() &&
                            gear.root_form_radius_mm.has_value();
    double q_root_1 = 0.0;
    double q_root_2 = 0.0;
    if (form_known) {
        q_root_1 = involute_line_offset(
                        std::max(pinion.base_radius_mm,
                                 *pinion.root_form_radius_mm),
                        pinion.base_radius_mm)
                       .value_or(0.0);
        q_root_2 = involute_line_offset(
                        std::max(gear.base_radius_mm,
                                 *gear.root_form_radius_mm),
                        gear.base_radius_mm)
                       .value_or(0.0);
    }

    double active_tip_q_1{};
    double active_tip_q_2{};
    double active_root_q_1{};
    double active_root_q_2{};
    if (!form_known) {
        const double tip_path_1 = *q_tip_1 - *q_work_1;
        const double tip_path_2 = *q_tip_2 - *q_work_2;
        active_tip_q_1 = *q_work_1 + tip_path_1;
        if (gear.internal) {
            active_tip_q_2 = *q_work_2 - (*q_work_2 - *q_tip_2);
            active_root_q_1 = *q_work_1 - (*q_work_2 - *q_tip_2);
            active_root_q_2 = *q_work_2 + tip_path_1;
        } else {
            active_tip_q_2 = *q_work_2 + tip_path_2;
            active_root_q_1 = *q_work_1 - tip_path_2;
            active_root_q_2 = *q_work_2 - tip_path_1;
        }
    } else if (gear.internal) {
        const double tip_path_1 = *q_tip_1 - *q_work_1;
        const double tip_path_2 = *q_work_2 - *q_tip_2;
        const double root_path_1 = *q_work_1 - q_root_1;
        const double root_path_2 = q_root_2 - *q_work_2;
        const double active_tip_path_1 = std::min(tip_path_1, root_path_2);
        const double active_tip_path_2 = std::min(tip_path_2, root_path_1);
        const double active_root_path_1 = std::min(root_path_1, tip_path_2);
        const double active_root_path_2 = std::min(root_path_2, tip_path_1);
        active_tip_q_1 = *q_work_1 + active_tip_path_1;
        active_tip_q_2 = *q_work_2 - active_tip_path_2;
        active_root_q_1 = *q_work_1 - active_root_path_1;
        active_root_q_2 = *q_work_2 + active_root_path_2;
    } else {
        const double tip_path_1 = *q_tip_1 - *q_work_1;
        const double tip_path_2 = *q_tip_2 - *q_work_2;
        const double root_path_1 = *q_work_1 - q_root_1;
        const double root_path_2 = *q_work_2 - q_root_2;
        const double active_tip_path_1 = std::min(tip_path_1, root_path_2);
        const double active_tip_path_2 = std::min(tip_path_2, root_path_1);
        const double active_root_path_1 = std::min(root_path_1, tip_path_2);
        const double active_root_path_2 = std::min(root_path_2, tip_path_1);
        active_tip_q_1 = *q_work_1 + active_tip_path_1;
        active_tip_q_2 = *q_work_2 + active_tip_path_2;
        active_root_q_1 = *q_work_1 - active_root_path_1;
        active_root_q_2 = *q_work_2 - active_root_path_2;
    }
    pinion.start_active_profile_radius_mm =
        radius_from_offset(pinion.base_radius_mm, active_root_q_1);
    pinion.active_tip_radius_mm =
        radius_from_offset(pinion.base_radius_mm, active_tip_q_1);
    gear.start_active_profile_radius_mm =
        radius_from_offset(gear.base_radius_mm, active_root_q_2);
    gear.active_tip_radius_mm =
        radius_from_offset(gear.base_radius_mm, active_tip_q_2);
}

} // namespace

namespace geargen::core::spur {

SpurParameters default_parameters(double module_mm, int z1, int z2)
{
    return SpurParameters::with_defaults(module_mm, z1, z2);
}

const MemberGeometry& SetGeometry::member(const std::string& which) const
{
    if (which == "pinion") {
        return pinion;
    }
    if (which == "gear") {
        return gear;
    }
    throw std::invalid_argument("member must be 'pinion' or 'gear'");
}

MemberGeometry& SetGeometry::member(const std::string& which)
{
    if (which == "pinion") {
        return pinion;
    }
    if (which == "gear") {
        return gear;
    }
    throw std::invalid_argument("member must be 'pinion' or 'gear'");
}

SetGeometry compute_set(const SpurSetParams& p)
{
    const double m_n = p.module;
    const double m_t = p.transverse_module();
    const double alpha_t = p.alpha_t();
    const double reference_distance = p.reference_centre_distance();
    const auto [working_alpha, working_distance] =
        working_geometry(p, reference_distance, alpha_t);
    const double tip_alteration =
        resolve_tip_alteration(p, working_distance);
    const double standard_thickness = kPi * m_t / 2.0;
    const auto target_backlash = working_backlash_target(p, working_alpha);

    auto make_member = [&](const std::string& name, int teeth, double beta,
                           bool internal, double profile_shift) {
        const double reference_radius = m_t * teeth / 2.0;
        const double base_radius = reference_radius * std::cos(alpha_t);
        const double working_radius = working_alpha == alpha_t
                                           ? reference_radius
                                           : base_radius / std::cos(working_alpha);
        const double shift_sign = internal ? -1.0 : 1.0;
        const double geometric_thickness = profile_shift == 0.0
                                                ? standard_thickness
                                                : m_t * (kPi / 2.0 +
                                                         shift_sign * 2.0 *
                                                             profile_shift *
                                                             std::tan(p.alpha_n()));
        double working_allowance{};
        double reference_allowance{};
        if (!target_backlash.has_value()) {
            working_allowance = p.backlash / 2.0 *
                                (working_radius / reference_radius);
            reference_allowance = p.backlash / 2.0;
        } else {
            working_allowance = *target_backlash *
                                (name == "pinion" ? p.backlash_allocation
                                                   : 1.0 - p.backlash_allocation);
            reference_allowance = working_allowance * reference_radius /
                                  working_radius;
        }
        const double reference_thickness =
            geometric_thickness - reference_allowance;
        const double normal_geometric = geometric_thickness * std::cos(p.beta());
        const double normal_thickness = reference_thickness * std::cos(p.beta());
        double addendum{};
        double dedendum{};
        if (internal) {
            addendum = m_n * (p.basic_rack_addendum_factor - profile_shift +
                              tip_alteration);
            dedendum = m_n * (p.basic_rack_dedendum_factor() + profile_shift);
        } else {
            addendum = m_n * (p.basic_rack_addendum_factor + profile_shift +
                              tip_alteration);
            dedendum = m_n * (p.basic_rack_dedendum_factor() - profile_shift);
        }
        const double tip_radius = internal ? reference_radius - addendum
                                            : reference_radius + addendum;
        const double root_radius = internal ? reference_radius + dedendum
                                             : reference_radius - dedendum;
        const double psi0 = internal
                                ? (kPi * m_t - reference_thickness) /
                                          (2.0 * reference_radius) + inv(alpha_t)
                                : reference_thickness / (2.0 * reference_radius) +
                                      inv(alpha_t);
        const double half_pitch = kPi / teeth;
        const double working_thickness = tooth_width_at_radius(
            internal, working_radius, base_radius, psi0, half_pitch);
        std::optional<double> generated_root;
        std::optional<double> root_form;
        std::optional<double> start_angle;
        std::optional<double> roll;
        std::optional<bool> undercut;
        if (p.root_geometry == "rack_generated" && !internal &&
            std::abs(beta) <= 1e-12) {
            const auto root = involute::rack_generated_root(
                reference_radius, base_radius, root_radius, psi0, half_pitch,
                dedendum, p.basic_rack_root_radius_factor * m_n);
            if (root.has_value()) {
                const auto soi = involute::solve_start_of_involute(*root);
                if (soi.has_value()) {
                    generated_root = root->generated_root_r();
                    root_form = soi->start_of_involute_r;
                    start_angle = soi->start_of_involute_angle;
                    roll = soi->involute_roll_parameter;
                    undercut = soi->undercut;
                }
            }
        }
        MemberGeometry result;
        result.name = name;
        result.teeth = teeth;
        result.beta_rad = beta;
        result.reference_radius_mm = reference_radius;
        result.base_radius_mm = base_radius;
        result.working_radius_mm = working_radius;
        result.tip_radius_mm = tip_radius;
        result.root_radius_mm = root_radius;
        result.tip_form_radius_mm = tip_radius;
        result.addendum_mm = addendum;
        result.dedendum_mm = dedendum;
        result.geometric_tooth_thickness_mm = geometric_thickness;
        result.reference_tooth_thickness_mm = reference_thickness;
        result.normal_geometric_tooth_thickness_mm = normal_geometric;
        result.normal_tooth_thickness_mm = normal_thickness;
        result.reference_tooth_thickness_allowance_mm = reference_allowance;
        result.working_tooth_thickness_mm = working_thickness;
        result.working_normal_tooth_thickness_mm =
            working_thickness * std::cos(working_helix_angle(beta, working_alpha));
        result.working_tooth_thickness_allowance_mm = working_allowance;
        result.virtual_teeth = teeth / std::pow(std::cos(beta), 3.0);
        result.twist_rad = p.face_width * std::tan(beta) / reference_radius + 0.0;
        result.psi0_rad = psi0;
        result.half_pitch_rad = half_pitch;
        result.internal = internal;
        result.profile_shift = profile_shift;
        result.generated_root_radius_mm = generated_root;
        result.root_form_radius_mm = root_form;
        result.start_of_involute_angle_rad = start_angle;
        result.involute_roll_parameter = roll;
        result.undercut = undercut;
        return result;
    };

    const double gear_beta = p.internal ? p.beta() : -p.beta();
    MemberGeometry pinion = make_member("pinion", p.z1, p.beta(), false,
                                        p.profile_shift_1);
    MemberGeometry gear = make_member("gear", p.z2, gear_beta, p.internal,
                                      p.profile_shift_2);
    resolve_active_profile_geometry(pinion, gear);
    const auto clearances = pair_tip_clearances(
        pinion, gear, working_distance, p.internal);
    const double circular_pitch = kPi * m_t;
    const double sin_beta = std::abs(std::sin(p.beta()));
    SetGeometry result{
        p,
        reference_distance,
        working_distance,
        alpha_t,
        working_alpha,
        m_t,
        circular_pitch,
        sin_beta > 1e-12 ? kPi * m_n / sin_beta
                         : std::numeric_limits<double>::infinity(),
        (p.basic_rack_addendum_factor + p.basic_rack_dedendum_factor()) * m_n,
        tip_alteration,
        pair_working_depth(pinion, gear, working_distance, p.internal),
        clearances.first,
        clearances.second,
        std::min(clearances.first, clearances.second),
        0.0,
        "approximate",
        0.0,
        sin_beta > 1e-12 ? p.face_width * sin_beta / (kPi * m_n) : 0.0,
        std::move(pinion),
        std::move(gear),
    };
    const auto active_branch = [](const MemberGeometry& member) {
        const double radius =
            member.active_tip_radius_mm.value_or(member.tip_radius_mm);
        return std::sqrt(std::max(
            0.0, radius * radius - member.base_radius_mm * member.base_radius_mm));
    };
    const double pinion_offset = active_branch(result.pinion) - std::sqrt(std::max(
        0.0, result.pinion.working_radius_mm * result.pinion.working_radius_mm -
                   result.pinion.base_radius_mm * result.pinion.base_radius_mm));
    const double gear_offset = active_branch(result.gear) - std::sqrt(std::max(
        0.0, result.gear.working_radius_mm * result.gear.working_radius_mm -
                   result.gear.base_radius_mm * result.gear.base_radius_mm));
    result.path_of_contact_mm = result.gear.internal
                                    ? pinion_offset - gear_offset
                                    : pinion_offset + gear_offset;
    result.contact_ratio_basis =
        result.pinion.root_form_radius_mm.has_value() &&
                result.gear.root_form_radius_mm.has_value()
            ? "active_profile"
            : "approximate";
    result.transverse_contact_ratio = transverse_contact_ratio(
        result.pinion, result.gear, result.working_centre_distance_mm,
        result.working_pressure_angle_rad, result.circular_pitch_mm,
        result.reference_pressure_angle_rad);
    return result;
}

SetGeometry derive(const SpurSetParams& parameters)
{
    return compute_set(parameters);
}

double resolve_tip_alteration(const SpurSetParams& p,
                              double working_centre_distance)
{
    if (p.tip_alteration_mode == "legacy") {
        return 0.0;
    }
    if (p.tip_alteration_mode == "explicit") {
        if (!p.tip_alteration_coefficient.has_value()) {
            throw std::invalid_argument(
                "explicit tip alteration mode requires a coefficient");
        }
        return *p.tip_alteration_coefficient;
    }
    if (p.tip_alteration_mode != "iso_clearance") {
        throw std::invalid_argument(
            "tip_alteration_mode must be legacy, iso_clearance, or explicit");
    }
    const double distance_change =
        (working_centre_distance - p.reference_centre_distance()) / p.module;
    return p.internal ? -distance_change + p.profile_shift_combination()
                      : distance_change - p.profile_shift_combination();
}

double pair_working_depth(const MemberGeometry& pinion,
                          const MemberGeometry& gear,
                          double working_centre_distance, bool internal)
{
    return internal ? working_centre_distance + pinion.tip_radius_mm -
                          gear.tip_radius_mm
                    : pinion.tip_radius_mm + gear.tip_radius_mm -
                          working_centre_distance;
}

std::pair<double, double> pair_tip_clearances(
    const MemberGeometry& pinion, const MemberGeometry& gear,
    double working_centre_distance, bool internal)
{
    if (internal) {
        return {root_clearance_radius(gear) - working_centre_distance -
                    pinion.tip_radius_mm,
                gear.tip_radius_mm - working_centre_distance -
                    pinion.root_radius_mm};
    }
    return {working_centre_distance - pinion.tip_radius_mm -
                root_clearance_radius(gear),
            working_centre_distance - gear.tip_radius_mm -
                root_clearance_radius(pinion)};
}

double transverse_contact_ratio(
    const MemberGeometry& pinion, const MemberGeometry& gear,
    double centre_distance, double alpha_t, double circular_pitch,
    std::optional<double> base_pitch_angle)
{
    const auto branch = [](const MemberGeometry& member) {
        return std::sqrt(std::max(
            0.0, member.tip_radius_mm * member.tip_radius_mm -
                       member.base_radius_mm * member.base_radius_mm));
    };
    const auto active_branch = [](const MemberGeometry& member) {
        const double radius =
            member.active_tip_radius_mm.value_or(member.tip_radius_mm);
        return std::sqrt(std::max(
            0.0, radius * radius - member.base_radius_mm * member.base_radius_mm));
    };
    double length{};
    if (pinion.active_tip_radius_mm.has_value() &&
        gear.active_tip_radius_mm.has_value()) {
        const double pinion_offset = active_branch(pinion) - std::sqrt(std::max(
            0.0, pinion.working_radius_mm * pinion.working_radius_mm -
                       pinion.base_radius_mm * pinion.base_radius_mm));
        const double gear_offset = active_branch(gear) - std::sqrt(std::max(
            0.0, gear.working_radius_mm * gear.working_radius_mm -
                       gear.base_radius_mm * gear.base_radius_mm));
        length = gear.internal ? pinion_offset - gear_offset
                               : pinion_offset + gear_offset;
    } else if (gear.internal) {
        length = branch(pinion) - branch(gear) + centre_distance * std::sin(alpha_t);
    } else {
        length = branch(pinion) + branch(gear) - centre_distance * std::sin(alpha_t);
    }
    const double pitch_angle = base_pitch_angle.value_or(alpha_t);
    return std::max(0.0, length / (circular_pitch * std::cos(pitch_angle)));
}

double min_internal_teeth(double alpha_t, double addendum_factor)
{
    const double denominator = 1.0 - std::cos(alpha_t);
    return denominator <= 0.0 ? std::numeric_limits<double>::infinity()
                              : 2.0 * addendum_factor / denominator;
}

double undercut_limit(double alpha_t, double beta, double profile_shift,
                      double addendum_factor,
                      std::optional<double> dedendum_factor,
                      double root_radius_factor)
{
    if (std::abs(beta) <= 1e-12 && dedendum_factor.has_value()) {
        const double effective_depth =
            *dedendum_factor - profile_shift -
            root_radius_factor * (1.0 - std::sin(alpha_t));
        return 2.0 * effective_depth / (std::sin(alpha_t) * std::sin(alpha_t));
    }
    return 2.0 * std::cos(beta) * (addendum_factor - profile_shift) /
           (std::sin(alpha_t) * std::sin(alpha_t));
}

double end_overshoot(const SetGeometry& geometry)
{
    return std::max(0.5, 0.05 * geometry.parameters.face_width);
}

int section_count(const SetGeometry& geometry, const std::string& member,
                  double max_sagitta)
{
    const auto& m = geometry.member(member);
    const double twist = std::abs(m.twist_rad);
    if (twist <= 0.0 || max_sagitta <= 0.0) {
        return 2;
    }
    const double ratio = 1.0 - max_sagitta / m.tip_radius_mm;
    if (ratio <= -1.0) {
        return 2;
    }
    const double step = 2.0 * std::acos(std::clamp(ratio, -1.0, 1.0));
    return std::max(2, static_cast<int>(std::ceil(twist / step)) + 1);
}

std::vector<double> section_heights(const SetGeometry& geometry,
                                    const std::string& member,
                                    double max_sagitta)
{
    const double overshoot = end_overshoot(geometry);
    const double low = -overshoot;
    const double high = geometry.parameters.face_width + overshoot;
    const int count = section_count(geometry, member, max_sagitta);
    std::vector<double> heights;
    heights.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        heights.push_back(low + (high - low) * i / (count - 1.0));
    }
    return heights;
}

Point3 to_axial_3d(double x, double y, double phase, double z) noexcept
{
    const double c = std::cos(phase);
    const double s = std::sin(phase);
    return {x * c - y * s, x * s + y * c, z};
}

double phase_at(const SetGeometry& geometry, const std::string& member,
                double z)
{
    const double face_width = geometry.parameters.face_width;
    return face_width <= 0.0 ? 0.0
                             : geometry.member(member).twist_rad * z / face_width;
}

ToothSpaceSection tooth_space_section(const SetGeometry& geometry,
                                      const std::string& member, double z,
                                      int flank_count, bool split_cap)
{
    const auto& p = geometry.parameters;
    const auto& m = geometry.member(member);
    double tip_radius{};
    double cap_radius{};
    if (m.internal) {
        tip_radius = std::max(
            m.tip_radius_mm,
            involute::min_internal_tip_radius(
                m.base_radius_mm, m.psi0_rad, m.half_pitch_rad,
                involute::kMinTopLandFactor * p.module));
        cap_radius = std::max(0.05 * m.base_radius_mm,
                              tip_radius - involute::kCutOvershootFactor * p.module);
    } else {
        tip_radius = std::min(
            m.tip_radius_mm,
            involute::max_tip_radius(
                m.base_radius_mm, m.psi0_rad,
                involute::kMinTopLandFactor * p.module));
        cap_radius = tip_radius + involute::kCutOvershootFactor * p.module;
    }

    std::optional<involute::RackRootEnvelope> rack_root;
    std::optional<double> generated_root;
    std::optional<double> root_form;
    std::optional<double> start_angle;
    std::optional<double> roll;
    std::optional<bool> undercut;
    if (p.root_geometry == "rack_generated" && !m.internal &&
        std::abs(p.beta()) <= 1e-12) {
        const auto root = involute::rack_generated_root(
            m.reference_radius_mm, m.base_radius_mm, m.root_radius_mm,
            m.psi0_rad, m.half_pitch_rad, m.dedendum_mm,
            p.basic_rack_root_radius_factor * p.module);
        if (root.has_value()) {
            const auto soi = involute::solve_start_of_involute(*root);
            if (soi.has_value() && tip_radius > soi->start_of_involute_r + 1e-10) {
                rack_root = involute::RackRootEnvelope{
                    root->sample_to(soi->trochoid_parameter,
                                    std::max(24, flank_count / 2)),
                    soi->start_of_involute_r};
                generated_root = root->generated_root_r();
                root_form = soi->start_of_involute_r;
                start_angle = soi->start_of_involute_angle;
                roll = soi->involute_roll_parameter;
                undercut = soi->undercut;
            }
        }
    }
    const auto loop = involute::tooth_space_loop(
        m.base_radius_mm, m.root_radius_mm, tip_radius, cap_radius, m.psi0_rad,
        m.half_pitch_rad, p.fillet_factor * p.module, flank_count, split_cap,
        m.internal, rack_root);
    if (loop.find_segment("generated_root_pos") == nullptr ||
        loop.find_segment("generated_root_pos")->empty()) {
        generated_root.reset();
        root_form.reset();
        start_angle.reset();
        roll.reset();
        undercut.reset();
    }
    return ToothSpaceSection{
        member,
        z,
        phase_at(geometry, member, z),
        m.root_radius_mm,
        tip_radius,
        cap_radius,
        loop.filleted,
        generated_root,
        root_form,
        start_angle,
        roll,
        undercut,
        loop.segments,
        loop.loop,
    };
}

bool ToothSpaceSection::rack_generated() const noexcept
{
    for (const auto& segment : segments) {
        if (segment.name == "generated_root_pos") {
            return !segment.points.empty();
        }
    }
    return false;
}

std::vector<Point3> ToothSpaceSection::loop_3d() const
{
    std::vector<Point3> result;
    result.reserve(loop_2d.size());
    for (const auto point : loop_2d) {
        result.push_back(to_axial_3d(point.x, point.y, phase_rad, z_mm));
    }
    return result;
}

std::vector<std::pair<std::string, std::vector<Point3>>>
ToothSpaceSection::segments_3d() const
{
    std::vector<std::pair<std::string, std::vector<Point3>>> result;
    result.reserve(segments.size());
    for (const auto& segment : segments) {
        std::vector<Point3> points;
        points.reserve(segment.points.size());
        for (const auto point : segment.points) {
            points.push_back(to_axial_3d(point.x, point.y, phase_rad, z_mm));
        }
        result.emplace_back(segment.name, std::move(points));
    }
    return result;
}

std::vector<Point3> guide_helix(const SetGeometry& geometry,
                                const std::string& member, double z_lo,
                                double z_hi, int points)
{
    if (points < 2) {
        throw std::invalid_argument("at least two guide points are required");
    }
    const auto section = tooth_space_section(geometry, member, 0.0);
    std::vector<Point3> result;
    result.reserve(static_cast<std::size_t>(points));
    for (int i = 0; i < points; ++i) {
        const double z = z_lo + (z_hi - z_lo) * i / (points - 1.0);
        result.push_back(to_axial_3d(section.cap_radius_mm, 0.0,
                                     phase_at(geometry, member, z), z));
    }
    return result;
}

double rim_radius(const SetGeometry& geometry, const std::string& member)
{
    const auto& m = geometry.member(member);
    return m.root_radius_mm + std::max(0.0, geometry.parameters.rim_thickness);
}

std::vector<Point2> blank_outline(const SetGeometry& geometry,
                                  const std::string& member)
{
    const auto& p = geometry.parameters;
    const auto& m = geometry.member(member);
    const double width = p.face_width;
    if (m.internal) {
        const double outer = rim_radius(geometry, member);
        return {{m.tip_radius_mm, 0.0}, {outer, 0.0}, {outer, width},
                {m.tip_radius_mm, width}};
    }
    const double bore = p.bore / 2.0;
    const double hub = std::max(bore, std::min(m.root_radius_mm,
                                               bore + 2.0 * p.module));
    std::vector<Point2> result{{bore, 0.0}, {m.tip_radius_mm, 0.0},
                               {m.tip_radius_mm, width}};
    if (p.hub_thickness > 0.0 && hub > bore + 1e-9) {
        result.push_back({hub, width});
        result.push_back({hub, width + p.hub_thickness});
        result.push_back({bore, width + p.hub_thickness});
    } else {
        result.push_back({bore, width});
    }
    return result;
}

} // namespace geargen::core::spur
