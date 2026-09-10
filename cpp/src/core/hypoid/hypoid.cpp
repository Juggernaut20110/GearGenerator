#include "core/hypoid/hypoid.hpp"

#include "core/bevel/bevel.hpp"
#include "core/common/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace geargen::core::hypoid {
namespace {

constexpr double kPreliminaryWheelFactor = 1.2;
constexpr double kDerivativeStepRad = 1e-6;
constexpr double kPhaseToleranceRad = 1e-11;
constexpr int kPhaseMaxDepth = 20;
constexpr double kCenterlineClearanceFactor = 1e-6;

double clamp_unit(double x) noexcept { return std::clamp(x, -1.0, 1.0); }

const MemberGeometry& get_member(const SetGeometry& geo, const std::string& name)
{
    if (name == "pinion") return geo.pinion;
    if (name == "gear") return geo.gear;
    throw std::invalid_argument("hypoid member must be 'pinion' or 'gear'");
}

double checked_asin(double value, const char* label)
{
    if (!std::isfinite(value) || std::abs(value) > 1.0)
        throw std::domain_error(std::string("hypoid Method 1 ") + label +
                                " is outside asin domain");
    return std::asin(value);
}

double checked_external(double numerator, double denominator, const char* label)
{
    const double angle = std::atan2(numerator, denominator);
    if (!(angle > 0.0 && angle < kPi / 2.0))
        throw std::domain_error(std::string("hypoid Method 1 ") + label +
                                " is outside the external-pair range");
    return angle;
}

double checked_tangent(double tangent, const char* label)
{
    if (!std::isfinite(tangent))
        throw std::domain_error(std::string("hypoid Method 1 ") + label +
                                " is not finite");
    return checked_external(tangent, 1.0, label);
}

std::pair<double, double> generated_angles(const HypoidSetParams& p,
                                           double limit)
{
    const double drive = p.alpha() + limit;
    const double coast = p.alpha() - limit;
    if (!(drive > 0.0 && drive < kPi / 2.0) ||
        !(coast > 0.0 && coast < kPi / 2.0))
        throw std::domain_error("Method 1 generated pressure angle is invalid");
    return {drive, coast};
}

struct Trial {
    double wheel_offset_axial{};
    double intermediate_offset_axial{};
    double intermediate_pitch{};
    double intermediate_offset_pitch{};
    double intermediate_spiral{};
    double dimension_increment{};
    double radius_increment{};
    double pinion_offset_axial{};
    double pinion_offset_pitch{};
    double pinion_spiral{};
    double wheel_spiral{};
    double pinion_pitch{};
    double wheel_pitch{};
    double pinion_radius{};
    double wheel_radius{};
    double pinion_cone{};
    double wheel_cone{};
    double limit_pressure{};
    double limit_radius{};
};

Trial trial(const HypoidSetParams& p, double offset, double delta_sigma,
            double desired_beta, double preliminary_dimension,
            double preliminary_pinion_radius, double wheel_mean_radius,
            double eta)
{
    const double ratio = p.ratio();
    const double intermediate_offset = checked_asin(
        (offset - preliminary_pinion_radius * std::sin(eta)) /
            wheel_mean_radius,
        "intermediate pinion axial offset angle");
    const double intermediate_pitch = checked_tangent(
        std::sin(eta) / (std::tan(intermediate_offset) * std::cos(delta_sigma)) +
            std::tan(delta_sigma) * std::cos(eta),
        "intermediate pinion pitch angle");
    const double intermediate_pitch_offset = checked_asin(
        std::sin(intermediate_offset) * std::cos(delta_sigma) /
            std::cos(intermediate_pitch),
        "intermediate pinion pitch-plane offset angle");
    const double intermediate_beta = std::atan2(
        preliminary_dimension - std::cos(intermediate_pitch_offset),
        std::sin(intermediate_pitch_offset));
    const double dimension_increment = std::sin(intermediate_pitch_offset) *
        (std::tan(desired_beta) - std::tan(intermediate_beta));
    const double radius_increment = wheel_mean_radius * dimension_increment / ratio;
    const double pinion_offset = checked_asin(
        std::sin(intermediate_offset) - radius_increment / wheel_mean_radius *
            std::sin(eta), "pinion axial offset angle");
    const double pinion_pitch = checked_tangent(
        std::sin(eta) / (std::tan(pinion_offset) * std::cos(delta_sigma)) +
            std::tan(delta_sigma) * std::cos(eta),
        "pinion pitch angle");
    const double pinion_pitch_offset = checked_asin(
        std::sin(pinion_offset) * std::cos(delta_sigma) /
            std::cos(pinion_pitch), "pinion pitch-plane offset angle");
    const double pinion_beta = std::atan2(
        preliminary_dimension + dimension_increment -
            std::cos(pinion_pitch_offset), std::sin(pinion_pitch_offset));
    const double wheel_beta = pinion_beta - pinion_pitch_offset;
    const double wheel_pitch = checked_tangent(
        std::sin(pinion_offset) / (std::tan(eta) * std::cos(delta_sigma)) +
            std::cos(pinion_offset) * std::tan(delta_sigma),
        "wheel pitch angle");
    const double pinion_radius = preliminary_pinion_radius + radius_increment;
    if (pinion_radius <= 0.0)
        throw std::domain_error("hypoid Method 1 pinion mean pitch radius is not positive");
    const double pinion_cone = pinion_radius / std::sin(pinion_pitch);
    const double wheel_cone = wheel_mean_radius / std::sin(wheel_pitch);
    const double limit_pressure = std::atan(
        -std::tan(pinion_pitch) * std::tan(wheel_pitch) *
        (pinion_cone * std::sin(pinion_beta) - wheel_cone * std::sin(wheel_beta)) /
        (std::cos(pinion_pitch_offset) *
         (pinion_cone * std::tan(pinion_pitch) + wheel_cone * std::tan(wheel_pitch))));
    const double denominator =
        -std::tan(limit_pressure) *
            (std::tan(pinion_beta) / (pinion_cone * std::tan(pinion_pitch)) +
             std::tan(wheel_beta) / (wheel_cone * std::tan(wheel_pitch))) +
        1.0 / (pinion_cone * std::cos(pinion_beta)) -
        1.0 / (wheel_cone * std::cos(wheel_beta));
    if (denominator == 0.0)
        throw std::domain_error("hypoid Method 1 curvature equation is singular");
    const double limit_radius = (1.0 / std::cos(limit_pressure)) *
        (std::tan(pinion_beta) - std::tan(wheel_beta)) / denominator;
    if (!std::isfinite(limit_radius) || limit_radius <= 0.0)
        throw std::domain_error("hypoid Method 1 limit radius of curvature is invalid");
    return {eta, intermediate_offset, intermediate_pitch, intermediate_pitch_offset,
            intermediate_beta, dimension_increment, radius_increment, pinion_offset,
            pinion_pitch_offset, pinion_beta, wheel_beta, pinion_pitch, wheel_pitch,
            pinion_radius, wheel_mean_radius, pinion_cone, wheel_cone,
            limit_pressure, limit_radius};
}

Method1Geometry pitch_solution(const HypoidSetParams& p)
{
    const double sigma = p.sigma();
    if (!(sigma > 0.0 && sigma < kPi))
        throw std::domain_error("shaft angle must be between 0 and 180 degrees");
    const double ratio = p.ratio();
    const double sign = p.offset >= 0.0 ? 1.0 : -1.0;
    const double desired = std::abs(p.psi1());
    const double delta_sigma = sigma - kPi / 2.0;
    if (std::abs(p.offset) < 1e-12) {
        const double d1 = std::atan2(std::sin(sigma), ratio + std::cos(sigma));
        const double d2 = sigma - d1;
        if (!(d1 > 0.0 && d1 < kPi / 2.0 && d2 > 0.0 && d2 < kPi / 2.0))
            throw std::domain_error("zero-offset pitch cones are outside the external-pair range");
        const double R2 = p.wheel_outer_radius() / std::sin(d2) - p.face_width / 2.0;
        if (R2 <= 0.0) throw std::domain_error("zero-offset mean cone distance is not positive");
        const double r2 = R2 * std::sin(d2);
        const double r1 = r2 / ratio;
        const auto [drive, coast] = generated_angles(p, 0.0);
        Method1Geometry result{};
        result.gear_ratio = ratio;
        result.desired_pinion_spiral_angle_rad = p.psi1();
        result.shaft_angle_departure_rad = delta_sigma;
        result.preliminary_wheel_pitch_angle_rad = d2;
        result.preliminary_wheel_mean_radius_mm = r2;
        result.preliminary_dimension_factor = 1.0;
        result.preliminary_pinion_mean_radius_mm = r1;
        result.intermediate_pinion_pitch_angle_rad = d1;
        result.intermediate_pinion_spiral_angle_rad = desired;
        result.pinion_spiral_angle_rad = desired;
        result.wheel_spiral_angle_rad = desired;
        result.pinion_pitch_angle_rad = d1;
        result.wheel_pitch_angle_rad = d2;
        result.pinion_mean_radius_mm = r1;
        result.wheel_mean_radius_mm = r2;
        result.pinion_mean_cone_distance_mm = R2;
        result.wheel_mean_cone_distance_mm = R2;
        result.generated_drive_normal_pressure_angle_rad = drive;
        result.generated_coast_normal_pressure_angle_rad = coast;
        result.iterations = 0;
        return result;
    }
    if (!p.cutter_radius.has_value())
        throw std::domain_error("non-zero-offset Method 1 geometry requires cutter_radius");
    if (*p.cutter_radius <= 0.0)
        throw std::domain_error("cutter radius must be greater than zero");
    const double offset = std::abs(p.offset);
    const double preliminary_wheel_angle = checked_external(
        ratio * std::cos(delta_sigma),
        kPreliminaryWheelFactor * (1.0 - ratio * std::sin(delta_sigma)),
        "preliminary wheel pitch angle");
    const double preliminary_wheel_radius =
        (p.wheel_outer_diameter() - p.face_width * std::sin(preliminary_wheel_angle)) / 2.0;
    if (preliminary_wheel_radius <= 0.0)
        throw std::domain_error("preliminary wheel mean pitch radius is not positive");
    const double preliminary_offset = checked_asin(
        offset * std::sin(preliminary_wheel_angle) / preliminary_wheel_radius,
        "preliminary pitch-plane offset angle");
    const double preliminary_dimension =
        std::tan(desired) * std::sin(preliminary_offset) +
        std::cos(preliminary_offset);
    const double preliminary_pinion_radius =
        preliminary_wheel_radius * preliminary_dimension / ratio;
    if (preliminary_pinion_radius <= 0.0)
        throw std::domain_error("preliminary pinion mean pitch radius is not positive");
    double eta = checked_external(
        offset,
        preliminary_wheel_radius *
            (std::tan(preliminary_wheel_angle) * std::cos(delta_sigma) -
             std::sin(delta_sigma)) + preliminary_pinion_radius,
        "initial wheel axial offset angle");
    Trial current{};
    int iteration = 0;
    double residual = 0.0;
    for (iteration = 1; iteration <= kMethod1MaxIterations; ++iteration) {
        current = trial(p, offset, delta_sigma, desired, preliminary_dimension,
                        preliminary_pinion_radius, preliminary_wheel_radius, eta);
        residual = current.limit_radius - *p.cutter_radius;
        if (std::abs(residual) <= kMethod1CurvatureToleranceMm) break;
        Trial ahead;
        try {
            ahead = trial(p, offset, delta_sigma, desired, preliminary_dimension,
                          preliminary_pinion_radius, preliminary_wheel_radius,
                          eta + kDerivativeStepRad);
        } catch (const std::exception&) {
            throw std::domain_error("hypoid Method 1 curvature iteration left the valid geometry domain");
        }
        const double derivative = (ahead.limit_radius - current.limit_radius) /
                                  kDerivativeStepRad;
        if (!std::isfinite(derivative) || std::abs(derivative) < 1e-12)
            throw std::domain_error("hypoid Method 1 curvature closure became singular");
        const double next_eta = eta - residual / derivative;
        if (!(next_eta > 0.0 && next_eta < kPi / 2.0))
            throw std::domain_error("hypoid Method 1 curvature closure left the valid axial-offset range");
        eta = next_eta;
    }
    if (iteration > kMethod1MaxIterations)
        throw std::domain_error("hypoid Method 1 curvature closure did not converge");
    if (std::abs(residual) > kMethod1CurvatureToleranceMm)
        throw std::domain_error("hypoid Method 1 curvature closure did not meet tolerance");
    const auto [drive, coast] = generated_angles(p, current.limit_pressure);
    auto signed_value = [sign](double value) { return sign * value; };
    Method1Geometry result{};
    result.gear_ratio = ratio;
    result.desired_pinion_spiral_angle_rad = p.psi1();
    result.shaft_angle_departure_rad = delta_sigma;
    result.preliminary_wheel_pitch_angle_rad = preliminary_wheel_angle;
    result.preliminary_wheel_mean_radius_mm = preliminary_wheel_radius;
    result.preliminary_pinion_offset_angle_rad = signed_value(preliminary_offset);
    result.preliminary_dimension_factor = preliminary_dimension;
    result.preliminary_pinion_mean_radius_mm = preliminary_pinion_radius;
    result.wheel_offset_angle_axial_rad = signed_value(current.wheel_offset_axial);
    result.intermediate_pinion_offset_angle_axial_rad = signed_value(current.intermediate_offset_axial);
    result.intermediate_pinion_pitch_angle_rad = current.intermediate_pitch;
    result.intermediate_pinion_offset_angle_pitch_rad = signed_value(current.intermediate_offset_pitch);
    result.intermediate_pinion_spiral_angle_rad = current.intermediate_spiral;
    result.dimension_factor_increment = current.dimension_increment;
    result.pinion_mean_radius_increment_mm = current.radius_increment;
    result.pinion_offset_angle_axial_rad = signed_value(current.pinion_offset_axial);
    result.pinion_offset_angle_pitch_rad = signed_value(current.pinion_offset_pitch);
    result.pinion_spiral_angle_rad = current.pinion_spiral;
    result.wheel_spiral_angle_rad = current.wheel_spiral;
    result.pinion_pitch_angle_rad = current.pinion_pitch;
    result.wheel_pitch_angle_rad = current.wheel_pitch;
    result.pinion_mean_radius_mm = current.pinion_radius;
    result.wheel_mean_radius_mm = current.wheel_radius;
    result.pinion_mean_cone_distance_mm = current.pinion_cone;
    result.wheel_mean_cone_distance_mm = current.wheel_cone;
    result.pitch_plane_offset_mm = signed_value(
        current.wheel_cone * std::sin(current.pinion_offset_pitch));
    result.limit_pressure_angle_rad = current.limit_pressure;
    result.generated_drive_normal_pressure_angle_rad = drive;
    result.generated_coast_normal_pressure_angle_rad = coast;
    result.limit_radius_of_curvature_mm = current.limit_radius;
    result.mean_tooth_curvature_mm = p.cutter_radius;
    result.curvature_residual_mm = residual;
    result.iterations = iteration;
    return result;
}

struct Depth {
    double basic_addendum_factor{};
    double basic_dedendum_factor{};
    double profile_shift{};
    double working_depth{};
    double clearance{};
    double whole_depth{};
    double pinion_addendum{};
    double pinion_dedendum{};
    double gear_addendum{};
    double gear_dedendum{};
};

Depth depth(const HypoidSetParams& p, double mn)
{
    if (p.depth_factor <= 0.0) throw std::domain_error("Method 1 depth factor must be greater than zero");
    if (p.clearance_factor < 0.0) throw std::domain_error("Method 1 clearance factor cannot be negative");
    if (!(p.gear_mean_addendum_factor > 0.0 && p.gear_mean_addendum_factor < 1.0))
        throw std::domain_error("Method 1 gear mean addendum factor must be between zero and one");
    const double hap = 0.5 * p.depth_factor;
    const double hfp = 0.5 * p.depth_factor + 2.0 * p.clearance_factor;
    const double x = p.depth_factor * (0.5 - p.gear_mean_addendum_factor);
    const double working = 2.0 * hap * mn;
    const double clearance = (hfp - hap) * mn;
    const double whole = (hap + hfp) * mn;
    const double pa = mn * (hap + x);
    const double pd = mn * (hfp - x);
    const double ga = mn * (hap - x);
    const double gd = mn * (hfp + x);
    for (double value : {working, clearance, whole, pa, pd, ga, gd})
        if (!std::isfinite(value) || value <= 0.0)
            throw std::domain_error("Method 1 tooth-depth inputs produce non-positive dimensions");
    return {hap, hfp, x, working, clearance, whole, pa, pd, ga, gd};
}

ThicknessGeometry thickness(const HypoidSetParams& p, double mn, double R2,
                            double Re2, double beta2)
{
    if (mn <= 0.0 || R2 <= 0.0 || Re2 <= 0.0 || std::abs(std::cos(p.alpha())) < 1e-12)
        throw std::domain_error("Method 1 tooth-thickness inputs are singular");
    const double outer = p.backlash;
    if (!std::isfinite(outer) || outer < 0.0)
        throw std::domain_error("outer transverse backlash must be finite and non-negative");
    const double mean_transverse = outer * R2 / Re2;
    const double mean_normal = mean_transverse * std::abs(std::cos(beta2));
    const double backlash_x = mean_normal /
        (4.0 * mn * std::cos(p.alpha()));
    const double theoretical = 0.5 * p.thickness_factor;
    return {p.alpha(), theoretical, backlash_x, theoretical - backlash_x,
            -theoretical - backlash_x, outer, mean_transverse, mean_normal};
}

MemberGeometry make_member(
    const std::string& name, int z, double delta, double radius,
    double cone_distance, double spiral, const HypoidSetParams& p,
    double tooth_module, double addendum, double dedendum, double working,
    double clearance, double whole, double addendum_angle,
    double dedendum_angle, double inner_cone, double face_width,
    double outer_face_width, double inner_face_width, double pitch_apex_z,
    double mean_pitch_z, double face_apex_z, double root_apex_z,
    double profile_shift, double thickness_modification,
    double generated_drive, double generated_coast,
    double inner_spiral, double outer_spiral)
{
    const double cd = std::cos(delta);
    const double sd = std::sin(delta);
    if (!(delta > 0.0 && delta < kPi / 2.0) || std::abs(cd) < 1e-12)
        throw std::domain_error(name + " Method 1 pitch angle is outside the external-pair range");
    if (cone_distance <= 0.0 || inner_cone <= 0.0 || face_width <= 0.0 ||
        outer_face_width <= 0.0 || inner_face_width <= 0.0)
        throw std::domain_error(name + " Method 1 member dimensions are not positive");
    const double outer_cone = cone_distance + outer_face_width;
    const double face_angle = delta + addendum_angle;
    const double root_angle = delta - dedendum_angle;
    const double outer_pitch_dia = 2.0 * outer_cone * sd;
    const double inner_pitch_dia = 2.0 * inner_cone * sd;
    const double outer_addendum = addendum + outer_face_width * std::tan(addendum_angle);
    const double inner_addendum = addendum - inner_face_width * std::tan(addendum_angle);
    const double outer_dedendum = dedendum + outer_face_width * std::tan(dedendum_angle);
    const double inner_dedendum = dedendum - inner_face_width * std::tan(dedendum_angle);
    for (double value : {outer_addendum, inner_addendum, outer_dedendum, inner_dedendum})
        if (!std::isfinite(value) || value <= 0.0)
            throw std::domain_error(name + " Method 1 face boundaries produce non-positive tooth depth");
    const double mean_tip = radius + addendum * cd;
    const double mean_root = radius - dedendum * cd;
    const double inner_tip = inner_pitch_dia / 2.0 + inner_addendum * cd;
    const double outer_tip = outer_pitch_dia / 2.0 + outer_addendum * cd;
    const double inner_root = inner_pitch_dia / 2.0 - inner_dedendum * cd;
    const double outer_root = outer_pitch_dia / 2.0 - outer_dedendum * cd;
    const double inner_tip_z = mean_pitch_z - inner_face_width * cd - inner_addendum * sd;
    const double outer_tip_z = mean_pitch_z + outer_face_width * cd - outer_addendum * sd;
    const double inner_root_z = mean_pitch_z - inner_face_width * cd + inner_dedendum * sd;
    const double outer_root_z = mean_pitch_z + outer_face_width * cd + outer_dedendum * sd;
    const double virtual_pitch = radius / cd;
    const double virtual_base = virtual_pitch * std::cos(
        normal_to_transverse_pressure_angle(p.alpha(), spiral));
    const double virtual_tip = virtual_pitch + addendum;
    const double virtual_root = virtual_pitch - dedendum;
    if (virtual_root <= 0.0) throw std::domain_error(name + " Method 1 Tredgold root radius is not positive");
    const double normal_thickness = 0.5 * tooth_module *
        (kPi + 2.0 * ((name == "gear" ? thickness_modification - profile_shift
                                        : thickness_modification + profile_shift) *
                       std::tan(p.alpha())));
    const double spiral_cos = std::abs(std::cos(spiral));
    if (normal_thickness <= 0.0 || spiral_cos < 1e-12)
        throw std::domain_error(name + " Method 1 tooth thickness is invalid");
    return {name, z, delta, radius, cone_distance, outer_cone, inner_cone,
            outer_cone, spiral, inner_spiral, outer_spiral, generated_drive,
            generated_coast, addendum, dedendum, working, clearance, whole,
            addendum_angle, dedendum_angle, face_angle, root_angle, face_width,
            outer_face_width + inner_face_width, outer_face_width, inner_face_width,
            inner_cone, pitch_apex_z, mean_pitch_z, face_apex_z, root_apex_z,
            inner_tip_z, outer_tip_z, inner_root_z, outer_root_z,
            outer_pitch_dia, inner_pitch_dia, 2.0 * outer_tip, 2.0 * inner_tip,
            outer_addendum, inner_addendum, outer_dedendum, inner_dedendum,
            outer_addendum + outer_dedendum, inner_addendum + inner_dedendum,
            mean_tip, mean_root, inner_tip, outer_tip, inner_root, outer_root,
            2.0 * outer_root, 2.0 * inner_root, static_cast<double>(z) / cd,
            virtual_pitch, virtual_base, virtual_tip, virtual_root,
            thickness_modification, normal_thickness,
            normal_thickness / spiral_cos, virtual_tip * cd, virtual_root * cd,
            virtual_root * outer_cone / cone_distance * cd};
}

struct BoundarySpirals {
    double pinion_boundary_wheel_outer{};
    double pinion_boundary_wheel_inner{};
    double pinion_inner{};
    double pinion_outer{};
    double wheel_inner{};
    double wheel_outer{};
};

double trace_angle(const HypoidSetParams& p, double mean_radius,
                   double mean_angle, double radius, const char* label)
{
    if (!p.cutter_radius.has_value()) return mean_angle;
    const auto trace = bevel::CrownTrace::for_set(
        std::abs(mean_angle), *p.cutter_radius, mean_radius, p.spiral_sign());
    if (!trace.reaches(radius, radius))
        throw std::domain_error(std::string("cutter radius does not reach ") + label);
    return trace.spiral_angle_at(radius);
}

BoundarySpirals boundary_spirals(const HypoidSetParams& p, double R1, double R2,
                                  double zeta_mp, double pitch_plane_offset,
                                  double beta1, double beta2, double be1,
                                  double bi1, double be2, double bi2)
{
    (void)beta1;
    const double cos_zeta = std::cos(std::abs(zeta_mp));
    const double Re21 = std::sqrt(R2 * R2 + be1 * be1 + 2.0 * R2 * be1 * cos_zeta);
    const double Ri21 = std::sqrt(R2 * R2 + bi1 * bi1 - 2.0 * R2 * bi1 * cos_zeta);
    const double wi = trace_angle(p, R2, beta2, R2 - bi2, "wheel inner-face");
    const double wo = trace_angle(p, R2, beta2, R2 + be2, "wheel outer-face");
    const double wpi = trace_angle(p, R2, beta2, Ri21, "pinion inner-face correspondence");
    const double wpo = trace_angle(p, R2, beta2, Re21, "pinion outer-face correspondence");
    double pi = 0.0, po = 0.0;
    if (std::abs(pitch_plane_offset) <= 1e-12) {
        pi = trace_angle(p, R1, beta1, R1 - bi1, "pinion inner-face");
        po = trace_angle(p, R1, beta1, R1 + be1, "pinion outer-face");
    } else {
        if (std::abs(pitch_plane_offset) > Ri21 || std::abs(pitch_plane_offset) > Re21)
            throw std::domain_error("Method 1 pitch-plane offset exceeds a longitudinal boundary");
        pi = wpi + p.spiral_sign() * std::asin(std::abs(pitch_plane_offset) / Ri21);
        po = wpo + p.spiral_sign() * std::asin(std::abs(pitch_plane_offset) / Re21);
    }
    return {Re21, Ri21, pi, po, wi, wo};
}

} // namespace

HypoidSetParams default_parameters(double module_mm, int z1, int z2)
{
    return HypoidSetParams::with_defaults(module_mm, z1, z2);
}

double normal_to_transverse_pressure_angle(double normal, double spiral)
{
    if (!std::isfinite(normal) || !(normal > -kPi / 2.0 && normal < kPi / 2.0))
        throw std::domain_error("normal pressure angle must lie between -90 and 90 degrees");
    const double cosine = std::abs(std::cos(spiral));
    if (cosine < 1e-12) throw std::domain_error("transverse pressure angle is singular");
    return std::atan2(std::tan(normal), cosine);
}

double MemberGeometry::angular_pitch_rad() const noexcept
{
    return kTau / static_cast<double>(z);
}

double MemberGeometry::mean_transverse_module_mm() const noexcept
{
    return 2.0 * pitch_radius_mm / z;
}

double MemberGeometry::outer_transverse_module_mm() const noexcept
{
    return outer_pitch_diameter_mm / z;
}

const MemberGeometry& SetGeometry::member(const std::string& which) const
{
    return get_member(*this, which);
}

double SetGeometry::circular_pitch_mm() const noexcept
{
    return kPi * mean_normal_module_mm;
}

SetGeometry derive(const HypoidSetParams& p)
{
    const Method1Geometry raw = pitch_solution(p);
    const double d1 = raw.pinion_pitch_angle_rad;
    const double d2 = raw.wheel_pitch_angle_rad;
    const double beta1 = p.spiral_sign() * raw.pinion_spiral_angle_rad;
    const double beta2 = p.spiral_sign() * raw.wheel_spiral_angle_rad;
    const double r1 = raw.pinion_mean_radius_mm;
    const double r2 = raw.wheel_mean_radius_mm;
    const double R1 = raw.pinion_mean_cone_distance_mm;
    const double R2 = raw.wheel_mean_cone_distance_mm;
    const double mn = 2.0 * r2 * std::cos(beta2) / p.z2;
    const auto d = depth(p, mn);
    const double Re2 = p.wheel_outer_radius() / std::sin(d2);
    const double be2 = Re2 - R2;
    const double bi2 = p.face_width - be2;
    if (be2 <= 0.0 || bi2 <= 0.0)
        throw std::domain_error("Method 1 wheel outer diameter and face width do not contain the wheel calculation point");
    const double Ri2 = R2 - bi2;
    if (Ri2 <= 0.0) throw std::domain_error("Method 1 wheel inner cone distance is not positive");
    const double cbe2 = be2 / p.face_width;
    const auto thick = thickness(p, mn, R2, Re2, beta2);
    const double delta_sigma = raw.shaft_angle_departure_rad;
    const double zeta_m = std::abs(raw.pinion_offset_angle_axial_rad);
    const double dm1 = 2.0 * R1 * std::sin(d1);
    const double dm2 = 2.0 * R2 * std::sin(d2);
    const double tzm2 = dm1 * std::sin(d2) / (2.0 * std::cos(d1)) -
        0.5 * std::cos(zeta_m) * std::tan(delta_sigma) *
        (dm2 + dm1 * std::cos(d2) / std::cos(d1));
    const double tzm1 = dm2 / 2.0 * std::cos(zeta_m) * std::cos(delta_sigma) -
                        tzm2 * std::sin(delta_sigma);
    const double tz1 = R1 * std::cos(d1) - tzm1;
    const double tz2 = R2 * std::cos(d2) - tzm2;
    const double theta_a2 = degrees_to_radians(p.gear_addendum_angle);
    const double theta_f2 = degrees_to_radians(p.gear_dedendum_angle);
    const double delta_a2 = d2 + theta_a2;
    const double delta_f2 = d2 - theta_f2;
    const double offset = std::abs(p.offset);
    const double den_root = R2 * std::cos(theta_f2) - tz2 * std::cos(delta_f2);
    const double den_face = R2 * std::cos(theta_a2) - tz2 * std::cos(delta_a2);
    if (std::abs(den_root) < 1e-12 || std::abs(den_face) < 1e-12)
        throw std::domain_error("Method 1 root/face angle calculation is singular");
    const double phi_r = std::atan2(offset * std::tan(delta_sigma) * std::cos(theta_f2), den_root);
    const double phi_o = std::atan2(offset * std::tan(delta_sigma) * std::cos(theta_a2), den_face);
    const double zeta_r = checked_asin(offset * std::cos(phi_r) * std::sin(delta_f2) / den_root,
                                       "pinion root-plane offset angle") - phi_r;
    const double zeta_o = checked_asin(offset * std::cos(phi_o) * std::sin(delta_a2) / den_face,
                                       "pinion face-plane offset angle") - phi_o;
    const double delta_a1 = checked_asin(
        std::sin(delta_sigma) * std::sin(delta_f2) +
        std::cos(delta_sigma) * std::cos(delta_f2) * std::cos(zeta_r),
        "pinion face angle");
    const double delta_f1 = checked_asin(
        std::sin(delta_sigma) * std::sin(delta_a2) +
        std::cos(delta_sigma) * std::cos(delta_a2) * std::cos(zeta_o),
        "pinion root angle");
    const double theta_a1 = delta_a1 - d1;
    const double theta_f1 = d1 - delta_f1;
    const double tzF2 = tz2 - (R2 * std::sin(theta_a2) - d.gear_addendum * std::cos(theta_a2)) /
                        std::sin(delta_a2);
    const double tzR2 = tz2 + (R2 * std::sin(theta_f2) - d.gear_dedendum * std::cos(theta_f2)) /
                        std::sin(delta_f2);
    const double tzF1 = (offset * std::sin(zeta_r) * std::cos(delta_f2) -
                         tzR2 * std::sin(delta_f2) - d.clearance) / std::sin(delta_a1);
    const double tzR1 = (offset * std::sin(zeta_o) * std::cos(delta_a2) -
                         tzF2 * std::sin(delta_a2) - d.clearance) / std::sin(delta_f1);
    const double denominator_lambda = p.ratio() * std::cos(d1) +
        std::cos(d2) * std::cos(std::abs(raw.pinion_offset_angle_pitch_rad));
    const double lambda_prime = std::atan2(
        std::sin(std::abs(raw.pinion_offset_angle_pitch_rad)) * std::cos(d2),
        denominator_lambda);
    const double cos_lambda = std::cos(std::abs(raw.pinion_offset_angle_pitch_rad) - lambda_prime);
    const double breri1 = p.face_width * std::cos(lambda_prime) / cos_lambda;
    const double delta_bx1 = d.working_depth *
        std::sin(std::abs(zeta_r)) * (1.0 - 1.0 / p.ratio());
    const double delta_gxe = cbe2 * breri1 * std::cos(delta_a1) / std::cos(theta_a1) +
        delta_bx1 - (d.gear_dedendum - d.clearance) * std::sin(d1);
    const double delta_gxi = (1.0 - cbe2) * breri1 * std::cos(delta_a1) /
        std::cos(theta_a1) + delta_bx1 +
        (d.gear_dedendum - d.clearance) * std::sin(d1);
    const double be1 = (delta_gxe + d.pinion_addendum * std::sin(d1)) *
                       std::cos(theta_a1) / std::cos(delta_a1);
    const double bi1 = (delta_gxi - d.pinion_addendum * std::sin(d1)) /
                       (std::cos(d1) - std::tan(theta_a1) * std::sin(d1));
    if (be1 <= 0.0 || bi1 <= 0.0) throw std::domain_error("Method 1 pinion face-width closure is not positive");
    const double Ri1 = R1 - bi1;
    if (Ri1 <= 0.0) throw std::domain_error("Method 1 pinion inner cone distance is not positive");
    const auto bs = boundary_spirals(p, R1, R2, raw.pinion_offset_angle_pitch_rad,
                                     raw.pitch_plane_offset_mm, beta1, beta2,
                                     be1, bi1, be2, bi2);
    Method1Geometry m1 = raw;
    m1.wheel_face_width_factor = cbe2;
    m1.wheel_outer_face_width_mm = be2;
    m1.wheel_inner_face_width_mm = bi2;
    m1.pinion_face_width_mm = breri1;
    m1.pinion_face_width_increment_along_axis_mm = delta_bx1;
    m1.pinion_outer_face_width_mm = be1;
    m1.pinion_inner_face_width_mm = bi1;
    m1.pinion_boundary_wheel_outer_cone_distance_mm = bs.pinion_boundary_wheel_outer;
    m1.pinion_boundary_wheel_inner_cone_distance_mm = bs.pinion_boundary_wheel_inner;
    m1.pinion_inner_spiral_angle_rad = bs.pinion_inner;
    m1.pinion_outer_spiral_angle_rad = bs.pinion_outer;
    m1.wheel_inner_spiral_angle_rad = bs.wheel_inner;
    m1.wheel_outer_spiral_angle_rad = bs.wheel_outer;
    m1.crossing_to_wheel_mean_z_mm = tzm2;
    m1.crossing_to_pinion_mean_z_mm = tzm1;
    m1.wheel_pitch_apex_z_mm = tz2;
    m1.pinion_pitch_apex_z_mm = tz1;
    m1.wheel_face_apex_z_mm = tzF2;
    m1.wheel_root_apex_z_mm = tzR2;
    m1.pinion_face_apex_z_mm = tzF1;
    m1.pinion_root_apex_z_mm = tzR1;
    m1.pinion_root_plane_offset_angle_rad = zeta_r;
    m1.pinion_face_plane_offset_angle_rad = zeta_o;
    m1.pinion_face_width_auxiliary_angle_rad = lambda_prime;
    const auto pinion = make_member(
        "pinion", p.z1, d1, r1, R1, beta1, p, mn, d.pinion_addendum,
        d.pinion_dedendum, d.working_depth, d.clearance, d.whole_depth,
        theta_a1, theta_f1, Ri1, breri1, be1, bi1, tz1, tzm1, tzF1, tzR1,
        d.profile_shift, thick.pinion_thickness_modification,
        m1.generated_drive_normal_pressure_angle_rad,
        m1.generated_coast_normal_pressure_angle_rad,
        bs.pinion_inner, bs.pinion_outer);
    const auto gear = make_member(
        "gear", p.z2, d2, r2, R2, beta2, p, mn, d.gear_addendum,
        d.gear_dedendum, d.working_depth, d.clearance, d.whole_depth,
        theta_a2, theta_f2, Ri2, p.face_width, be2, bi2, tz2, tzm2, tzF2, tzR2,
        d.profile_shift, thick.gear_thickness_modification,
        m1.generated_drive_normal_pressure_angle_rad,
        m1.generated_coast_normal_pressure_angle_rad,
        bs.wheel_inner, bs.wheel_outer);
    return {p, gear.outer_cone_distance_mm, gear.cone_distance_mm,
            gear.inner_cone_distance_mm, mn, d.basic_addendum_factor,
            d.basic_dedendum_factor, d.profile_shift, d.working_depth,
            d.clearance, d.whole_depth, raw.pinion_offset_angle_pitch_rad,
            raw.pitch_plane_offset_mm, m1, thick, pinion, gear};
}

namespace {

std::vector<Point2> reflect_points(const std::vector<Point2>& points)
{
    std::vector<Point2> result;
    result.reserve(points.size());
    for (const auto& point : points) result.push_back({point.x, -point.y});
    return result;
}

void remove_duplicate_points(std::vector<Point2>& points)
{
    std::vector<Point2> result;
    for (const auto& point : points) {
        if (result.empty() || distance(result.back(), point) > 1e-9)
            result.push_back(point);
    }
    points = std::move(result);
}

std::pair<std::vector<Point2>, std::vector<Point2>> hypoid_root_fillet(
    const std::vector<Point2>& flank, double root, double rho, bool negative,
    bool strict)
{
    if (!std::isfinite(rho) || rho < 0.0)
        throw std::domain_error("hypoid root fillet radius must be finite and non-negative");
    const auto working = negative ? reflect_points(flank) : flank;
    auto result = involute::root_fillet(working, root, rho);
    if (!result.has_value()) {
        if (rho > 0.0 && strict)
            throw std::domain_error("hypoid root fillet radius does not fit flank");
        return {flank, {}};
    }
    auto trimmed = result->flank;
    auto arc = result->arc;
    if (negative) {
        trimmed = reflect_points(trimmed);
        arc = reflect_points(arc);
    }
    remove_duplicate_points(trimmed);
    remove_duplicate_points(arc);
    return {std::move(trimmed), std::move(arc)};
}

std::tuple<std::vector<involute::NamedSegment>, std::vector<Point2>, bool,
           std::vector<Point2>, std::vector<Point2>> hypoid_loop(
    std::vector<Point2> left, std::vector<Point2> right, double root,
    double cap_radius, double fillet_radius, bool split_cap, bool strict)
{
    if (left.size() < 2 || right.size() < 2)
        throw std::invalid_argument("hypoid flank curves need at least two points");
    if (root <= 0.0 || cap_radius <= root)
        throw std::domain_error("hypoid tooth-space root and cap radii are invalid");
    const auto left_result = hypoid_root_fillet(left, root, fillet_radius, true, strict);
    const auto right_result = hypoid_root_fillet(right, root, fillet_radius, false, strict);
    left = left_result.first;
    right = right_result.first;
    const auto& left_arc = left_result.second;
    const auto& right_arc = right_result.second;
    const auto root_point = [](const std::vector<Point2>& arc,
                               const std::vector<Point2>& flank) {
        return arc.empty() ? flank.front() : arc.front();
    };
    const double left_root = std::atan2(root_point(left_arc, left).y,
                                        root_point(left_arc, left).x);
    const double right_root = std::atan2(root_point(right_arc, right).y,
                                         root_point(right_arc, right).x);
    const double left_tip = std::atan2(left.back().y, left.back().x);
    const double right_tip = std::atan2(right.back().y, right.back().x);
    if (!(left_root < 0.0 && right_root > 0.0 && left_tip < 0.0 && right_tip > 0.0))
        throw std::domain_error("hypoid flanks do not bound the space centreline");
    std::vector<Point2> cap;
    for (const double angle : {left_tip, 0.5 * left_tip, 0.0,
                               0.5 * right_tip, right_tip})
        cap.push_back(involute::polar(cap_radius, angle));
    std::vector<Point2> root_arc;
    for (int i = 0; i < 9; ++i)
        root_arc.push_back(involute::polar(root,
            right_root + (left_root - right_root) * i / 8.0));
    std::vector<Point2> right_reversed(right.rbegin(), right.rend());
    std::vector<Point2> right_arc_reversed(right_arc.rbegin(), right_arc.rend());
    std::vector<involute::NamedSegment> segments{
        {"fillet_neg", left_arc}, {"flank_neg", left},
        {"riser_neg", {left.back(), cap.front()}}, {"cap", cap},
        {"riser_pos", {cap.back(), right.back()}},
        {"flank_pos", std::move(right_reversed)},
        {"fillet_pos", std::move(right_arc_reversed)},
        {"root", std::move(root_arc)}};
    std::vector<std::string> order{"fillet_neg", "flank_neg", "riser_neg",
                                   "cap", "riser_pos", "flank_pos",
                                   "fillet_pos", "root"};
    if (split_cap) {
        auto it = std::find_if(segments.begin(), segments.end(),
            [](const auto& item) { return item.name == "cap"; });
        const std::size_t mid = cap.size() / 2;
        std::vector<Point2> cap_neg(cap.begin(), cap.begin() + mid + 1);
        std::vector<Point2> cap_pos(cap.begin() + mid, cap.end());
        *it = {"cap_neg", std::move(cap_neg)};
        segments.insert(it + 1, {"cap_pos", std::move(cap_pos)});
        auto order_it = std::find(order.begin(), order.end(), "cap");
        *order_it = "cap_neg";
        order.insert(order_it + 1, "cap_pos");
    }
    std::vector<Point2> loop;
    for (const auto& name : order) {
        auto it = std::find_if(segments.begin(), segments.end(),
            [&](const auto& item) { return item.name == name; });
        if (it == segments.end()) continue;
        for (const auto& point : it->points)
            if (loop.empty() || distance(loop.back(), point) > 1e-9)
                loop.push_back(point);
    }
    if (loop.size() > 1 && distance(loop.front(), loop.back()) < 1e-9)
        loop.pop_back();
    return {std::move(segments), std::move(loop),
            !left_arc.empty() && !right_arc.empty(), std::move(left),
            std::move(right)};
}

std::vector<Point2> hypoid_flank(const MemberGeometry& m, double cone_dist,
                                 double normal_angle, double spiral_angle,
                                 double normal_thickness, int count,
                                 const char* label, bool construction_only)
{
    const double scale = cone_dist / std::max(m.cone_distance_mm, 1e-9);
    const double pitch = m.virtual_pitch_r_mm * scale;
    const double transverse_angle = normal_to_transverse_pressure_angle(
        normal_angle, spiral_angle);
    const double transverse_thickness = normal_thickness /
                                        std::max(std::abs(std::cos(spiral_angle)), 1e-12);
    const double base = pitch * std::cos(transverse_angle);
    const double root = m.virtual_root_r_mm * scale;
    const double tip = m.virtual_tip_r_mm * scale;
    const double half_pitch = kPi / m.virtual_teeth;
    const double tooth_half_angle = transverse_thickness / (2.0 * std::max(pitch, 1e-9));
    const double psi0 = tooth_half_angle + involute::inv(transverse_angle);
    auto points = involute::flank_points(base, root, tip, psi0, half_pitch, count);
    const double clearance = std::max(1e-8, kCenterlineClearanceFactor * pitch);
    double min_y = std::numeric_limits<double>::infinity();
    for (const auto& point : points) min_y = std::min(min_y, point.y);
    if (min_y <= clearance)
        throw std::domain_error(std::string(m.name) + " " + label +
                                (construction_only ? " construction-only" : " physical") +
                                " Tredgold flank reaches the centreline");
    remove_duplicate_points(points);
    return points;
}

struct TraceDistance { std::optional<bevel::CrownTrace> trace; double distance{}; };

TraceDistance phase_trace_distance(const MemberGeometry& member, double cone_dist,
                                   const SetGeometry& geo)
{
    if (!std::isfinite(cone_dist) || cone_dist <= 0.0)
        throw std::domain_error("hypoid phase cone distance must be finite and positive");
    const bool offset_pinion = member.name == "pinion" &&
                               std::abs(geo.pitch_plane_offset_mm) > 1e-12;
    const MemberGeometry& trace_member = offset_pinion ? geo.gear : member;
    std::optional<bevel::CrownTrace> trace;
    if (geo.parameters.cutter_radius.has_value()) {
        trace = bevel::CrownTrace::for_set(
            std::abs(trace_member.mean_spiral_angle_rad),
            *geo.parameters.cutter_radius, trace_member.cone_distance_mm,
            geo.parameters.spiral_sign());
    }
    double trace_dist = cone_dist;
    if (offset_pinion) {
        const double t = cone_dist - geo.pinion.cone_distance_mm;
        trace_dist = std::sqrt(geo.gear.cone_distance_mm * geo.gear.cone_distance_mm +
            t * t + 2.0 * geo.gear.cone_distance_mm * t *
            std::cos(std::abs(geo.method1.pinion_offset_angle_pitch_rad)));
    }
    if (trace.has_value()) {
        const double lo = std::abs(trace->centre_distance_mm - trace->cutter_radius_mm);
        const double hi = trace->centre_distance_mm + trace->cutter_radius_mm;
        const double tolerance = 1e-10 * std::max(1.0, hi);
        if (trace_dist < lo - tolerance || trace_dist > hi + tolerance)
            throw std::domain_error("hypoid phase requires cutter-trace distance outside valid domain");
    }
    return {trace, trace_dist};
}

double method1_spiral_angle_at(const MemberGeometry& member, double cone_dist,
                               const SetGeometry& geo)
{
    const auto [trace, trace_dist] = phase_trace_distance(member, cone_dist, geo);
    if (member.name == "gear" || std::abs(geo.pitch_plane_offset_mm) <= 1e-12)
        return trace.has_value() ? trace->spiral_angle_at(trace_dist)
                                 : member.mean_spiral_angle_rad;
    const double wheel_angle = trace.has_value() ? trace->spiral_angle_at(trace_dist)
                                                 : geo.gear.mean_spiral_angle_rad;
    const double ratio = std::abs(geo.pitch_plane_offset_mm) / trace_dist;
    if (ratio >= 1.0) throw std::domain_error("hypoid Method 1 pinion trace offset is singular");
    return wheel_angle + geo.parameters.spiral_sign() * std::asin(ratio);
}

double integrate_phase(const MemberGeometry& member, double start, double stop,
                       const SetGeometry& geo)
{
    if (start == stop) return 0.0;
    const bool reverse = stop < start;
    const double lo = reverse ? stop : start;
    const double hi = reverse ? start : stop;
    const double sd = std::sin(member.pitch_angle_rad);
    if (std::abs(sd) < 1e-12) throw std::domain_error("hypoid trace phase has a singular pitch angle");
    auto integrand = [&](double value) {
        const double beta = method1_spiral_angle_at(member, value, geo);
        const double result = std::tan(beta) / (value * sd);
        if (!std::isfinite(result)) throw std::domain_error("hypoid phase integrand is non-finite");
        return result;
    };
    auto simpson = [](double a, double b, double fa, double fb, double fc) {
        return (b - a) * (fa + 4.0 * fc + fb) / 6.0;
    };
    const double fa = integrand(lo), fb = integrand(hi);
    const double mid = 0.5 * (lo + hi), fm = integrand(mid);
    const double whole = simpson(lo, hi, fa, fb, fm);
    const double tolerance = kPhaseToleranceRad * (1.0 + std::abs(whole));
    std::function<double(double,double,double,double,double,double,int,double)> recurse;
    recurse = [&](double a, double b, double left_value, double right_value,
                  double center, double old, int depth_left, double local_tol) {
        const double mid_value = 0.5 * (a + b);
        const double left_mid = 0.5 * (a + mid_value);
        const double right_mid = 0.5 * (mid_value + b);
        const double fl = integrand(left_mid), fr = integrand(right_mid);
        const double left = simpson(a, mid_value, left_value, center, fl);
        const double right = simpson(mid_value, b, center, right_value, fr);
        const double refined = left + right;
        if (std::abs(refined - old) / 15.0 <= local_tol)
            return refined + (refined - old) / 15.0;
        if (depth_left <= 0) throw std::domain_error("hypoid phase integration did not converge");
        return recurse(a, mid_value, left_value, center, fl, left,
                       depth_left - 1, local_tol / 2.0) +
               recurse(mid_value, b, center, right_value, fr, right,
                       depth_left - 1, local_tol / 2.0);
    };
    const double result = recurse(lo, hi, fa, fb, fm, whole,
                                  kPhaseMaxDepth, tolerance);
    return reverse ? -result : result;
}

} // namespace

std::pair<double, double> contact_azimuths(const SetGeometry& geo)
{
    const double d1 = geo.pinion.pitch_angle_rad;
    const double d2 = geo.gear.pitch_angle_rad;
    const double sigma = geo.parameters.sigma();
    if (std::abs(std::sin(sigma)) < 1e-12)
        throw std::domain_error("hypoid contact azimuth is singular at zero shaft angle");
    const double nz = -std::sin(d1);
    const double nx = (std::sin(d2) + std::sin(d1) * std::cos(sigma)) /
                      std::sin(sigma);
    const double ny2 = 1.0 - nx * nx - nz * nz;
    if (ny2 < -1e-10)
        throw std::domain_error("hypoid pitch-surface normals do not share a contact direction");
    const double ny = std::copysign(std::sqrt(std::max(0.0, ny2)),
                                    geo.parameters.offset == 0.0 ? 1.0 : geo.parameters.offset);
    const double theta1 = std::atan2(ny / std::cos(d1), nx / std::cos(d1));
    const double axis2[3]{std::sin(sigma), 0.0, std::cos(sigma)};
    const double radial_x2[3]{std::cos(sigma), 0.0, -std::sin(sigma)};
    const double normal[3]{nx, ny, nz};
    double radial2[3]{};
    for (int i = 0; i < 3; ++i)
        radial2[i] = (-normal[i] + std::sin(d2) * axis2[i]) / std::cos(d2);
    const double theta2 = std::atan2(radial2[1], radial2[0] * radial_x2[0] +
                                      radial2[1] * radial_x2[1] + radial2[2] * radial_x2[2]);
    return {theta1, theta2};
}

double phase(const SetGeometry& geo, const std::string& name, double cone_dist,
             bool construction_only)
{
    const auto& m = geo.member(name);
    const double sd = std::sin(m.pitch_angle_rad);
    if (!std::isfinite(cone_dist) || cone_dist <= 0.0 || std::abs(sd) < 1e-12)
        throw std::domain_error("hypoid trace phase has a singular or invalid cone distance");
    const bool outside = cone_dist < m.tooth_face_inner_cone_distance_mm ||
                         cone_dist > m.tooth_face_outer_cone_distance_mm;
    if (construction_only && outside) {
        try {
            (void)phase_trace_distance(m, cone_dist, geo);
        } catch (const std::exception& error) {
            if (std::string(error.what()).find("outside valid domain") == std::string::npos)
                throw;
            const double boundary = cone_dist < m.tooth_face_inner_cone_distance_mm
                ? m.tooth_face_inner_cone_distance_mm
                : m.tooth_face_outer_cone_distance_mm;
            const double boundary_phase = phase(geo, name, boundary, false);
            const double slope = std::tan(method1_spiral_angle_at(m, boundary, geo)) /
                                 (boundary * sd);
            const double sense = name == "pinion" ? 1.0 : -1.0;
            return boundary_phase + sense * slope * (cone_dist - boundary);
        }
    }
    const auto [trace, trace_dist] = phase_trace_distance(m, cone_dist, geo);
    double curve = 0.0;
    const bool zero_offset = std::abs(geo.pitch_plane_offset_mm) <= 1e-12;
    if (trace.has_value() && (name == "gear" || zero_offset)) {
        curve = -trace->sign * trace->theta_at(trace_dist) / sd;
    } else if (!trace.has_value() && zero_offset) {
        curve = std::tan(m.mean_spiral_angle_rad) *
                std::log(cone_dist / m.cone_distance_mm) / sd;
    } else {
        curve = integrate_phase(m, m.cone_distance_mm, cone_dist, geo);
    }
    const double result = (name == "pinion" ? 1.0 : -1.0) * curve;
    if (!std::isfinite(result)) throw std::domain_error("hypoid phase is non-finite");
    return result;
}

Section tooth_space_section(const SetGeometry& geo, const std::string& name,
                            std::optional<double> requested_cone,
                            bool split_cap, int flank_points)
{
    const auto& m = geo.member(name);
    const double cone_dist = requested_cone.value_or(m.cone_distance_mm);
    const double scale = cone_dist / std::max(m.cone_distance_mm, 1e-9);
    const double root = m.virtual_root_r_mm * scale;
    const double tip = m.virtual_tip_r_mm * scale;
    const double cap = tip + involute::kCutOvershootFactor * geo.parameters.module;
    const double profile_cone = std::clamp(
        cone_dist, m.tooth_face_inner_cone_distance_mm,
        m.tooth_face_outer_cone_distance_mm);
    const double beta = method1_spiral_angle_at(m, profile_cone, geo);
    const double local_cos = std::abs(std::cos(beta));
    if (local_cos < 1e-12)
        throw std::domain_error("hypoid section tooth thickness is singular at a 90 degree spiral");
    const double transverse_thickness = m.mean_transverse_tooth_thickness_mm * scale;
    const double normal_thickness = transverse_thickness * local_cos;
    const bool positive_drive = geo.parameters.hand == Hand::Right;
    const double positive_normal = positive_drive
        ? m.generated_drive_normal_pressure_angle_rad
        : m.generated_coast_normal_pressure_angle_rad;
    const double negative_normal = positive_drive
        ? m.generated_coast_normal_pressure_angle_rad
        : m.generated_drive_normal_pressure_angle_rad;
    const double positive_transverse = normal_to_transverse_pressure_angle(positive_normal, beta);
    const double negative_transverse = normal_to_transverse_pressure_angle(negative_normal, beta);
    const double pitch = m.virtual_pitch_r_mm * scale;
    const double positive_base = pitch * std::cos(positive_transverse);
    const double negative_base = pitch * std::cos(negative_transverse);
    const bool construction_only =
        cone_dist < m.tooth_face_inner_cone_distance_mm ||
        cone_dist > m.tooth_face_outer_cone_distance_mm;
    std::vector<Point2> positive;
    std::vector<Point2> negative;
    try {
        positive = hypoid_flank(m, cone_dist, positive_normal, beta,
                                normal_thickness, flank_points,
                                positive_drive ? "drive" : "coast",
                                construction_only);
        negative = reflect_points(hypoid_flank(
            m, cone_dist, negative_normal, beta, normal_thickness, flank_points,
            positive_drive ? "coast" : "drive", construction_only));
    } catch (const std::exception&) {
        if (!construction_only) throw;
        const double boundary = cone_dist < m.tooth_face_inner_cone_distance_mm
            ? m.tooth_face_inner_cone_distance_mm
            : m.tooth_face_outer_cone_distance_mm;
        return tooth_space_section(geo, name, boundary, split_cap, flank_points);
    }
    const auto [segments, loop, filleted, left, right] = hypoid_loop(
        std::move(negative), std::move(positive), root, cap,
        geo.parameters.effective_root_fillet_radius(), split_cap,
        geo.parameters.root_fillet_radius.has_value());
    const double base_phase = phase(geo, name, cone_dist, construction_only);
    return {name, cone_dist, base_phase, m.pitch_angle_rad,
            cone_dist / std::max(std::cos(m.pitch_angle_rad), 1e-9), root, tip,
            cap, beta, normal_thickness, transverse_thickness,
            positive_drive ? positive_transverse : negative_transverse,
            positive_drive ? negative_transverse : positive_transverse,
            positive_drive ? positive_base : negative_base,
            positive_drive ? negative_base : positive_base,
            geo.parameters.effective_root_fillet_radius(), filleted,
            left, right, segments, loop};
}

std::vector<Point3> Section::loop_3d() const
{
    std::vector<Point3> result;
    result.reserve(loop.size());
    for (const auto& point : loop)
        result.push_back(bevel::to_cone_3d(point.x, point.y, pitch_angle_rad,
                                           cone_apex_z_mm, phase_rad));
    return result;
}

std::vector<Point2> blank_outline(const SetGeometry& geo, const std::string& name)
{
    const auto& m = geo.member(name);
    const auto& p = geo.parameters;
    const double bore = name == "pinion" ? p.bore / 2.0
                                        : std::max(0.5, p.bore / 2.0);
    if (m.tooth_face_inner_cone_distance_mm <= 0.0 ||
        m.tooth_face_outer_cone_distance_mm <= m.tooth_face_inner_cone_distance_mm)
        throw std::domain_error(name + " Method 1 tooth-face boundaries are invalid");
    const double z_back = m.outer_root_z_mm + std::max(0.0, p.min_root_thickness);
    std::vector<Point2> result{{bore, m.inner_tip_z_mm},
                               {m.inner_tip_radius_mm, m.inner_tip_z_mm},
                               {m.outer_tip_radius_mm, m.outer_tip_z_mm},
                               {m.outer_root_radius_mm, m.outer_root_z_mm}};
    if (p.min_root_thickness > 0.0)
        result.push_back({m.outer_root_radius_mm, z_back});
    if (p.hub_thickness > 0.0) {
        const double hub = std::min(m.outer_root_radius_mm * 0.7,
                                    bore + 2.0 * std::max(p.module, 1.0));
        result.push_back({hub, z_back});
        result.push_back({hub, z_back + p.hub_thickness});
        result.push_back({bore, z_back + p.hub_thickness});
    } else result.push_back({bore, z_back});
    return result;
}

SectionBounds section_cone_bounds(const SetGeometry& geo, const std::string& name)
{
    const auto& m = geo.member(name);
    const auto outline = blank_outline(geo, name);
    const double front = outline.front().y;
    const std::size_t back_index = geo.parameters.min_root_thickness > 0.0 ? 4 : 3;
    const double back = outline.at(back_index).y;
    const double margin = std::max(0.1, 0.1 * geo.parameters.module);
    const double step = std::max(geo.parameters.module,
                                 m.face_width_along_pitch_cone_mm / 8.0);
    auto clears_front = [&](double cone) {
        const auto section = tooth_space_section(geo, name, cone);
        double max_z = -std::numeric_limits<double>::infinity();
        for (const auto& point : section.loop_3d()) max_z = std::max(max_z, point.z);
        return max_z < front - margin;
    };
    auto clears_back = [&](double cone) {
        const auto section = tooth_space_section(geo, name, cone);
        double min_z = std::numeric_limits<double>::infinity();
        for (const auto& point : section.loop_3d()) min_z = std::min(min_z, point.z);
        return min_z > back + margin;
    };
    double loft_inner = m.tooth_face_inner_cone_distance_mm;
    if (!clears_front(loft_inner)) {
        double hi = loft_inner, lo = hi - step;
        for (int i = 0; i < 100 && !clears_front(lo); ++i) {
            if (lo <= 0.0) throw std::domain_error("could not clear hypoid blank front face");
            hi = lo;
            lo -= step;
        }
        for (int i = 0; i < 60; ++i) {
            const double mid = 0.5 * (lo + hi);
            if (clears_front(mid)) lo = mid; else hi = mid;
        }
        loft_inner = lo;
    }
    double loft_outer = m.tooth_face_outer_cone_distance_mm;
    if (!clears_back(loft_outer)) {
        double lo = loft_outer, hi = lo + step;
        for (int i = 0; i < 100 && !clears_back(hi); ++i) {
            lo = hi;
            hi += step;
        }
        for (int i = 0; i < 60; ++i) {
            const double mid = 0.5 * (lo + hi);
            if (clears_back(mid)) hi = mid; else lo = mid;
        }
        loft_outer = hi;
    }
    return {m.cone_distance_mm, m.tooth_face_inner_cone_distance_mm,
            m.tooth_face_outer_cone_distance_mm, loft_inner, loft_outer};
}

std::vector<double> section_cone_distances(const SetGeometry& geo,
                                           const std::string& name, int count)
{
    const auto bounds = section_cone_bounds(geo, name);
    const int n = std::max(2, count);
    std::vector<double> result;
    for (int i = 0; i < n; ++i)
        result.push_back(bounds.loft_inner_mm +
                         (bounds.loft_outer_mm - bounds.loft_inner_mm) * i /
                         std::max(n - 1, 1));
    for (double value : {bounds.tooth_face_inner_mm, bounds.calculation_point_mm,
                         bounds.tooth_face_outer_mm}) {
        if (value > bounds.loft_inner_mm && value < bounds.loft_outer_mm)
            result.push_back(value);
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end(),
                             [](double a, double b) { return std::abs(a - b) <= 1e-12; }),
                 result.end());
    return result;
}

int section_count(const SetGeometry& geo, const std::string& name)
{
    const auto& m = geo.member(name);
    const auto bounds = section_cone_bounds(geo, name);
    const double inner_phase = phase(geo, name, bounds.loft_inner_mm, true);
    const double outer_phase = phase(geo, name, bounds.loft_outer_mm, true);
    const double sagitta = std::max(0.01, 0.02 * geo.parameters.module);
    const double step = 2.0 * std::acos(clamp_unit(
        1.0 - sagitta / std::max(m.tredgold_tip_radius_mm, 1e-9)));
    const int minimum = std::max(2, static_cast<int>(std::ceil(
        std::abs(outer_phase - inner_phase) / std::max(step, 1e-9))) + 1);
    return static_cast<int>(section_cone_distances(geo, name, minimum).size());
}

} // namespace geargen::core::hypoid
